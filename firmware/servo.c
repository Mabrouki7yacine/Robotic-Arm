#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.h"

#define TAG "servo"

inline int usToTicks(int usec){
    return (usec * 8192)/20000; // Adjusted for 13-bit resolution (2^13 = 8192)
}

void attach_servo(int servo, int pin) {
    int timer_num;

    switch (servo) {
        case 0:
        case 1:
            timer_num = 0;
            break;
        case 2:
        case 3:
            timer_num = 1;
            break;
        case 4:
            timer_num = 2;
            break;
        default:
            ESP_LOGE(TAG, "Invalid servo index");
            return;
    }
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = timer_num,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = servo,
        .timer_sel      = timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void servo_angle(int servo, int angle) {
    int duty = (2000 / 180) * angle + 500;
    if (duty > 2500) {
        duty = 2500;
    }

    if (duty < 500) {
        duty = 500;
    }

    esp_err_t err = ledc_set_duty(LEDC_MODE, servo, usToTicks(duty));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed: %s", esp_err_to_name(err));
        return;
    }

    err = ledc_update_duty(LEDC_MODE, servo);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed: %s", esp_err_to_name(err));
        return;
    }
}

inline void servo_write(int servo,int angle){
    if (angle < 0) angle = 0;   
    if (angle > 180) angle = 180;
    for (int i = 0; i < 4; i++){
        servo_angle(servo, angle);
        vTaskDelay(7 / portTICK_PERIOD_MS);
    }
}

void go_smooth(int servo, int angle){
    static int last_angle[2] = {-1, -1};
    int last;
    if(servo < 0 || servo > 1){
        ESP_LOGE(TAG, "Ya kho using only two servo's");
        return;
    }
    last = last_angle[servo];
    if(last == -1){
        servo_angle(servo, angle);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }else if (angle < last){
        for (int pos = last; pos >= angle; pos -= 1) {
            servo_angle(servo, pos);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }else if(angle > last){
        for (int pos = last; pos <= angle; pos += 1) { 
            servo_angle(servo, pos);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }else{
        servo_angle(servo, angle);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    last_angle[servo] = angle;
}
