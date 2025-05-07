#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "servo.h"
#include "wifi_connect.h"

int servo0Pin = 14;
int servo1Pin = 27;

int angle[2] = {90, 90};

#define PHONE

#ifdef OFFICE
    #define WIFI_SSID ""
    #define WIFI_PASSWORD ""
    #define IP_server ""
#elif defined PHONE
    #define WIFI_SSID ""
    #define WIFI_PASSWORD ""
    #define IP_server ""
#elif defined HOME
    #define WIFI_SSID ""
    #define WIFI_PASSWORD ""
    #define IP_server ""
#endif


#define PORT 3000
#define my_delay(time) vTaskDelay((time) / portTICK_PERIOD_MS)
#define TAG "ROBOTIC_ARM"
QueueHandle_t queue;
int32_t message[2];

void servo_control_task(){
    //first  joint: 095 middle
    //second joint: 085 middle
    //first  joint: 005 max
    //second joint: 175 max
    while (1) {
        if(xQueueReceive(queue, &(message), 0)){
            ESP_LOGI(TAG, "Moving servo 0 to %d",(int) message[0]);
            go_smooth(axe_0, message[0]);
            my_delay(1000);
            if(message[1]>180){
                message[1] = 360 - message[1];
            }            
            if (message[1] > 135) message[1] = 135;
            ESP_LOGI(TAG, "Moving servo 1 to %d",(int) message[1]);
            go_smooth(axe_1, message[1]);
            my_delay(500);
        }else{
            my_delay(50);  
        }/*
        ESP_LOGI(TAG, "Moving servo 0 to 120°");
        go_smooth(axe_0, 90);
        my_delay(1000);

        ESP_LOGI(TAG, "Moving servo 1 to 60°");
        go_smooth(axe_1, 0);
        my_delay(1000);

        ESP_LOGI(TAG, "Moving servo 0 to 120°");
        go_smooth(axe_0, 120);
        my_delay(1000);

        ESP_LOGI(TAG, "Moving servo 1 to 60°");
        go_smooth(axe_1, 90);
        my_delay(1000); */
    }
}

void tcp_conn_task(){
    while (true){
        int my_sock = -1;
        int error = -1;
        struct sockaddr_in dest_addr = {0};
        dest_addr.sin_len = 0;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        dest_addr.sin_addr.s_addr = inet_addr(IP_server);
        my_sock = socket(AF_INET, SOCK_STREAM, 0);

        if (my_sock < 0) {
            ESP_LOGE("TCP SOCKET", "Unable to create socket: errno %s", strerror(my_sock));
            return;   
        }
        else{
            ESP_LOGI("TCP SOCKET", "Successfully created socket with channel: %d", my_sock);
        }
        ESP_LOGI("TCP SOCKET", "Socket created, receiving from %s:%d", IP_server, PORT);

        while (true) {
            while (error != 0){
                error = connect(my_sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                if (error != 0) {
                    ESP_LOGE("TCP SOCKET", "Socket unable to connect: errno %s", strerror(errno));   
                }
                else{
                    ESP_LOGI("TCP SOCKET", "Successfully connected");
                }
                my_delay(1000);
            }
            // receiving data 
            ESP_LOGI("TCP SOCKET", "Waiting for data to recv");
            int32_t rx_buffer[2];
            int len = recv(my_sock, rx_buffer, sizeof(rx_buffer), 0);
            if (len < 0) {
                ESP_LOGE("TCP SOCKET", "Error occurred while sending data: errno %s", strerror(errno));
                my_delay(1000);
                break;
            } else {
                xQueueOverwrite(queue, rx_buffer);
                ESP_LOGI("TCP SOCKET", "Successfully received data angle_0: %d, angle_1: %d",(int)rx_buffer[0], (int)rx_buffer[1]);
            }
        }
    }

    vTaskDelete(NULL);
}

void app_main(void){ 
    attach_servo(axe_0,servo0Pin);
    attach_servo(axe_1,servo1Pin);
    queue = xQueueCreate(1, sizeof(message));
    if(queue == 0){
    	ESP_LOGE("QUEUE", "QUEUE CREATION FAILED");
    	return ;
    }
    wifi_connection(WIFI_SSID, WIFI_PASSWORD);
    xTaskCreate(tcp_conn_task, "TCP_client", 4096, NULL, 5, NULL);
    xTaskCreate(servo_control_task, "Servo_ctrl", 4096, NULL, 2, NULL);
}
