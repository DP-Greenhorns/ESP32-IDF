#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT UART_NUM_1
#define TXD_PIN   12   // ESP32 TX -> EC200U RX
#define RXD_PIN   13   // ESP32 RX <- EC200U TX
#define BUF_SIZE  2048

static const char *TAG = "EC200U_GNSS";

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized on port %d (TX:%d, RX:%d)", UART_PORT, TXD_PIN, RXD_PIN);
}

void send_cmd_and_wait(const char *cmd, uint8_t *response, int *resp_len, int wait_ms)
{
    // Clear UART buffer first
    uart_flush(UART_PORT);
    
    // Send command
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(UART_PORT, "\r\n", 2);
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // Wait for response
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    
    // Read response
    *resp_len = uart_read_bytes(UART_PORT, response, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
    if (*resp_len > 0) {
        response[*resp_len] = '\0';
        ESP_LOGI(TAG, "<< %s", response);
    }
}

void gnss_task(void *arg)
{
    uint8_t data[BUF_SIZE];
    int len = 0;
    int retry_count = 0;
    
    // Wait for module to be ready
    ESP_LOGI(TAG, "Waiting for EC200U module to initialize...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Step 1: Test AT communication
    ESP_LOGI(TAG, "Testing AT communication...");
    send_cmd_and_wait("AT", data, &len, 500);
    
    // Step 2: Check module info
    ESP_LOGI(TAG, "Checking module version...");
    send_cmd_and_wait("ATI", data, &len, 500);
    
    // Step 3: Turn off GNSS first (in case it was on)
    ESP_LOGI(TAG, "Ensuring GNSS is off first...");
    send_cmd_and_wait("AT+QGPSEND", data, &len, 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Step 4: Configure GNSS mode (GPS + GLONASS + Beidou + Galileo)
    ESP_LOGI(TAG, "Configuring GNSS to use all satellites...");
    send_cmd_and_wait("AT+QGPSCFG=\"gnssconfig\",5", data, &len, 500);
    
    // Step 5: Turn on GNSS with standalone mode
    ESP_LOGI(TAG, "Turning GNSS ON (standalone mode)...");
    send_cmd_and_wait("AT+QGPS=1", data, &len, 2000);
    
    // Step 6: Check GNSS power status
    ESP_LOGI(TAG, "Checking GNSS power status...");
    send_cmd_and_wait("AT+QGPS?", data, &len, 500);
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "GNSS Started - Waiting for satellite fix...");
    ESP_LOGI(TAG, "This may take 30-60 seconds for cold start");
    ESP_LOGI(TAG, "Make sure antenna has clear sky view!");
    ESP_LOGI(TAG, "===========================================");
    
    // Main loop
    while (1) {
        retry_count++;
        
        // Check satellite info every 5 seconds
        if (retry_count % 5 == 0) {
            ESP_LOGI(TAG, "--- Checking satellite visibility ---");
            send_cmd_and_wait("AT+QGPSGNMEA=\"GSV\"", data, &len, 1000);
        }
        
        // Request location using AT+QGPSLOC
        ESP_LOGI(TAG, "[Attempt %d] Requesting location...", retry_count);
        send_cmd_and_wait("AT+QGPSLOC=2", data, &len, 2000);
        
        // Parse response
        if (len > 0) {
            // Check for error code 516 (no fix yet)
            if (strstr((char*)data, "+CME ERROR: 516") != NULL) {
                ESP_LOGW(TAG, "No GNSS fix yet (Error 516). Satellites acquiring...");
            }
            // Check for error code 505 (GNSS session not started)
            else if (strstr((char*)data, "+CME ERROR: 505") != NULL) {
                ESP_LOGE(TAG, "GNSS not started! Restarting...");
                send_cmd_and_wait("AT+QGPS=1", data, &len, 2000);
            }
            // Check for successful location
            else if (strstr((char*)data, "+QGPSLOC:") != NULL) {
                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "✓ GNSS FIX ACQUIRED!");
                ESP_LOGI(TAG, "Location Data: %s", data);
                ESP_LOGI(TAG, "========================================");
                
                // Parse coordinates from response
                // Format: +QGPSLOC: <UTC>,<latitude>,<longitude>,<hdop>,<altitude>,<fix>,<cog>,<spkm>,<spkn>,<date>,<nsat>
                char *loc_start = strstr((char*)data, "+QGPSLOC:");
                if (loc_start) {
                    ESP_LOGI(TAG, "Parsed: %s", loc_start);
                }
            }
        }
        
        // Also try NMEA GGA sentence
        if (retry_count % 3 == 0) {
            ESP_LOGI(TAG, "Trying NMEA GGA format...");
            send_cmd_and_wait("AT+QGPSGNMEA=\"GGA\"", data, &len, 1000);
            
            if (len > 0 && strstr((char*)data, "$G") != NULL) {
                ESP_LOGI(TAG, "NMEA Data: %s", data);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "EC200U GNSS Location Tracker");
    ESP_LOGI(TAG, "====================================");
    
    uart_init();
    
    xTaskCreate(gnss_task, "gnss_task", 8192, NULL, 5, NULL);
}