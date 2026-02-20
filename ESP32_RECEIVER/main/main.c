#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// create a character array to store the data in char format
// data is stored as a char array with length 10.
// This will allow it the data value to be displayed on the screen later on
char data_rec[10];

static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint8_t *mac = recv_info->src_addr;

    sprintf(data_rec, "%d", *data); // fprmatting the incoming data value to fit into a string

    printf("Received from MAC %02X:%02X:%02X:%02X:%02X:%02X: %.*s\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len, (char *)data); // printing out values
}

// initialize wifi capabilities so that ESP32's can communicate over network
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
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

void app_main(void)
{
    wifi_init(); // initialize wifi function

    esp_now_init(); // initializing built-in esp-now functionality

    // Find local MAC address
    uint8_t mac_l[6] = {0};
    esp_read_mac(mac_l, ESP_MAC_WIFI_STA);
    printf("Local MAC address: \"0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\" \n", mac_l[0], mac_l[1], mac_l[2], mac_l[3], mac_l[4], mac_l[5]);

    // ESP-NOW
    esp_now_register_recv_cb(on_data_recv); // method to receive incoming data

    while (1)
    {
        printf("hi\n");
        vTaskDelay(pdMS_TO_TICKS(500)); // delat
    }
}
