#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           (13) // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 50 Hz

static const char* TAG = "servo";

typedef enum {base, axe_0, axe_1, axe_2, gripper} servo_index;

inline int usToTicks(int usec);
void attach_servo(int servo, int pin);
void servo_angle(int servo, int angle);

#endif