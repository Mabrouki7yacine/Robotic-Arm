#include "http_client.h"
#include <stdio.h>
#include <esp_log.h>

extern char readings[256];
extern int base_angle;
extern int axe_0_angle;
extern int axe_1_angle;
extern int axe_2_angle;
extern int gripper_angle;

#define phone

/*#ifdef home
    const char* url = "http://192.168.100.58:3000/arm";
#elif wameedh
    const char* url = "http://172.16.43.99:3000/arm";
#elif phone
    const char* url = "http://192.168.43.238:3000/arm";
#elif corso
    const char* url = "http://192.168.8.102:3000/arm";
#endif*/
    const char* url = "http://192.168.43.238:3000/arm";

void extract_values(const char *json_string) {
    const char *pos0 = strstr(json_string, "\"base\":");
    const char *pos1 = strstr(json_string, "\"axe_0\":");
    const char *pos2 = strstr(json_string, "\"axe_1\":");
    const char *pos3 = strstr(json_string, "\"axe_2\":");
    const char *pos4 = strstr(json_string, "\"gripper\":");

    // Extract the values using sscanf
    sscanf(pos0 , "\"base\":%d", &base_angle);
    sscanf(pos1 , "\"axe_0\":%d", &axe_0_angle);
    sscanf(pos2 , "\"axe_1\":%d", &axe_1_angle);
    sscanf(pos3 , "\"axe_2\":%d", &axe_2_angle);
    sscanf(pos4 , "\"gripper\":%d", &gripper_angle);
}

esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        // Ensure we don't write more than the buffer size
        if (evt->data_len < sizeof(readings)) {
            snprintf(readings, sizeof(readings), "%.*s", evt->data_len, (char *)evt->data);
            extract_values(readings);
        } else {
            ESP_LOGE("HTTP_CLIENT", "Data too long for buffer");
        }
        break;

    default:
        break;
    }
    return ESP_OK;
}
void get_request(){
    esp_http_client_config_t config_get = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_get_handler};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        ESP_LOGI("HTTP_CLIENT", "GET request successful");
    } else {
        ESP_LOGE("HTTP_CLIENT", "GET request failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
}
//0011110110000111111101110000001111101100001
//111101100001