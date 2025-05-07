
#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "esp_event.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"


void wifi_connection(const char* wifi_ssid, const char* wifi_pass);

#endif
