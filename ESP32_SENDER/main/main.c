/* VL53L0X Single Sensor Example
 * 
 * This example demonstrates basic usage of a single VL53L0X sensor
 * using the legacy API (vl53l0x_config)
 * 
 * Hardware connections:
 * - VL53L0X VCC -> 3.3V or 5V (depending on module)
 * - VL53L0X GND -> GND
 * - VL53L0X SCL -> GPIO 22 (or your chosen SCL pin)
 * - VL53L0X SDA -> GPIO 21 (or your chosen SDA pin)
 * - VL53L0X XSHUT -> GPIO 23 (optional, for power control)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "vl53l0x.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char *TAG = "VL53L0X_SINGLE"; //create TAG value. Used for debugging and appearing in logs

// I2C Configuration for time of flight sensor
#define I2C_MASTER_NUM      0
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_SDA_IO   21
#define VL53L0X_XSHUT_PIN   -1  // Set to -1 if not using XSHUT
#define VL53L0X_ADDRESS     0x29  // Default I2C address

//ESP-NOW Configuration

// Receiver MAC Address
//Change to the MAC address of the device that you will be sending information to
uint8_t mac_destination[6] = {0x8c, 0x4f, 0x00, 0x28, 0xc2, 0x04};

// Callback function for sending information
void on_data_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
     if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    printf("Sent to MAC %02X:%02X:%02X:%02X:%02X:%02X - Status: %s\n",
           tx_info->des_addr[0], tx_info->des_addr[1],tx_info->des_addr[2],
           tx_info->des_addr[3], tx_info->des_addr[4], tx_info->des_addr[5],
           status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void wifi_init()
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // WiFi on channel 1
}


//Main entry point
void app_main(void)
{
    /*******************************Setting up ESP-NOW Comms***********************************/
    //initializing wifi connection
    wifi_init();

    // Find local mac address
    //Change to the MAC address of the device that you are running THIS code on
    uint8_t my_esp_mac[6] = {0x90, 0x15, 0x06, 0x7C, 0x48, 0x98};
    esp_read_mac(my_esp_mac, ESP_MAC_WIFI_STA);
    printf("Local MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
           my_esp_mac[0], my_esp_mac[1], my_esp_mac[2],
           my_esp_mac[3], my_esp_mac[4], my_esp_mac[5]);

    // ESP-NOW initiation and register a callback function that will be called
    esp_now_init();
    esp_now_register_send_cb(on_data_sent);

    // Add a peer device with which ESP32 can communicate
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac_destination, 6);
    peer.channel = 1;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    ESP_LOGI(TAG, "VL53L0X Single Sensor Example"); //method for printing out logs related to ESP

    /******************Reading from VL53l0x Sensor***********************/
    
    // Initialize VL53L0X sensor with legacy API
    // This creates and manages its own I2C bus
    vl53l0x_t *sensor = vl53l0x_config(
        I2C_MASTER_NUM,
        I2C_MASTER_SCL_IO,
        I2C_MASTER_SDA_IO,
        VL53L0X_XSHUT_PIN,
        VL53L0X_ADDRESS,
        1  // Use 2.8V I/O mode (set to 0 for 1.8V)
    );
    
    if (!sensor) {
        ESP_LOGE(TAG, "Failed to configure VL53L0X sensor"); //prints if unable to configure VL53L0X sensor
        return;
    }
    
    ESP_LOGI(TAG, "VL53L0X configured successfully"); //prints if sensor configured successfully
    
    // Initialize the sensor
    const char *err = vl53l0x_init(sensor);
    if (err) {
        ESP_LOGE(TAG, "VL53L0X initialization failed: %s", err);
        vl53l0x_end(sensor);
        return;
    }
    
    ESP_LOGI(TAG, "VL53L0X initialized successfully");
    
    // Optional: Set timing budget (default is ~33ms)
    // Higher budget = more accurate but slower
    err = vl53l0x_setMeasurementTimingBudget(sensor, 50000);  // 50ms
    if (err) {
        ESP_LOGW(TAG, "Failed to set timing budget: %s", err);
    }
    
    // Mode 2: Continuous measurements
    ESP_LOGI(TAG, "\n=== Continuous Mode ===");
    ESP_LOGI(TAG, "Starting continuous ranging (200ms interval)...");

    uint16_t reading = 0; //variable to store sensor reading and send it to other ESP

    while(1){
          
        ESP_LOGI(TAG, "\n=== High-Speed Mode ===");
        ESP_LOGI(TAG, "Starting back-to-back continuous ranging...");
    
        vl53l0x_startContinuous(sensor, 0);  // 0 = back-to-back mode (fastest)
    
        for (int i = 0; i < 10; i++) {
            uint16_t range_mm = vl53l0x_readRangeContinuousMillimeters(sensor);
            ESP_LOGI(TAG, "Range: %d mm", range_mm);
            vTaskDelay(pdMS_TO_TICKS(50));  // Small delay

            reading = range_mm;
        }
    
        vl53l0x_stopContinuous(sensor);
        ESP_LOGI(TAG, "Continuous ranging stopped");

          //range_mm 
        char str[10];

        sprintf(str, "%d", reading);//formatting reading to char so it can be sent to other device
        const char *message = str;


        // Cleanup
        // vl53l0x_end(sensor);
        // ESP_LOGI(TAG, "Example finished");

        esp_now_send(mac_destination, (uint8_t *)message, strlen(message));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}