#include <stdio.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_client.h"
#include "servo.h"
#include "wifi_connect.h"
#include "http_client.h"

char readings[256];

int servo1Pin = 14;
int servo2Pin = 26;
int servo3Pin = 25;
int servo4Pin = 33;
int servo5Pin = 32;

int base_angle = 0;
int axe_0_angle = 90;
int axe_1_angle = 90;
int axe_2_angle = 90;
int gripper_angle = 90;

int last_axe_0_angle = 90;
int last_axe_1_angle = 90;
int last_axe_2_angle = 90;
int last_gripper_angle = 90;

void go_smooth(int servo, int angle , int last_angle){
    int ang;
    ang = angle - last_angle;
    if (ang < 0){
        for (int pos = last_angle; pos >= angle; pos -= 1) {
            for (int i = 0; i < 4; i++){
                servo_angle(servo,pos);
                //vTaskDelay(15 / portTICK_PERIOD_MS);
                vTaskDelay(7 / portTICK_PERIOD_MS);
            }
        }
    }else if(ang > 0){
        for (int posi = last_angle; posi <= angle; posi += 1) { 
            for (int i = 0; i < 4; i++){
                servo_angle(servo,posi);
                vTaskDelay(7 / portTICK_PERIOD_MS);
            }
        }
    }else{
        for (int i = 0; i <3; i++){
            servo_angle(servo,angle);
            vTaskDelay(7 / portTICK_PERIOD_MS);
        }
    }

}

void app_main(void){ 
    wifi_connection();
    attach_servo(base,servo1Pin);
    attach_servo(axe_0,servo2Pin);
    attach_servo(axe_1,servo3Pin);
    attach_servo(axe_2,servo4Pin);
    attach_servo(gripper,servo5Pin);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    printf("WIFI was initiated ...........\n\n");
    while (1) {
        get_request();
        // Now you can use the angles extracted from the GET request
        printf("Base Angle: %d\n", base_angle);
        printf("Axe 0 Angle: %d\n", axe_0_angle);
        printf("Axe 1 Angle: %d\n", axe_1_angle);
        printf("Axe 2 Angle: %d\n", axe_2_angle);
        printf("Gripper Angle: %d\n", gripper_angle);

        if (base_angle == 1){
            servo_angle(base, 70);
            vTaskDelay(125 / portTICK_PERIOD_MS);
            servo_angle(base, 90);
        }else if (base_angle == -1){
            servo_angle(base, 110);
            vTaskDelay(125 / portTICK_PERIOD_MS);
            servo_angle(base, 90);
        }else{
            servo_angle(base, 90);
        }

        go_smooth(axe_0,axe_0_angle,last_axe_0_angle);
        go_smooth(axe_1,axe_1_angle,last_axe_1_angle);
        go_smooth(axe_2,axe_2_angle,last_axe_2_angle);
        go_smooth(gripper,gripper_angle,gripper_angle);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        last_axe_0_angle = axe_0_angle ;
        last_axe_1_angle = axe_1_angle ;
        last_axe_2_angle = axe_2_angle ;
        last_gripper_angle = gripper_angle ;
    }
}
