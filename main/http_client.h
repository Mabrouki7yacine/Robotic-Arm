#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include <esp_http_client.h>

// Function declarations
void extract_values(const char *json_string);
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt);
void get_request();

#endif // HTTP_CLIENT_H