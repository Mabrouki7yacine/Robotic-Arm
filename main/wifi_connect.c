#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "wifi_connect.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"

#define phone

/*#ifdef HOME
    #define WIFI_SSID "AT_9HYtGXp"
    #define WIFI_PASS "zgu2ad0qfbh4"
#elif wameedh
    #define WIFI_SSID "Wameedh_Wifi"
    #define WIFI_PASS "WameedhC001A"
#elif phone
    #define WIFI_SSID "Honor 8X"
    #define WIFI_PASS "zgu2ad0qfbh4"
#endif*/

    #define WIFI_SSID "Honor 8X"
    #define WIFI_PASS "zgu2ad0qfbh4"
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        //delay = 5;
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation
    esp_event_loop_create_default();     // Event loop
    esp_netif_create_default_wifi_sta(); // WiFi station

    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); // WiFi initialization

    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        }
    };
    esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);
    
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();

    // 4 - Wi-Fi Connect Phase
    esp_wifi_connect();
}

