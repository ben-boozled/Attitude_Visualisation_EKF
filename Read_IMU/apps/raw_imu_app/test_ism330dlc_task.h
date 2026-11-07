/** 
 * @file test_ism330dlc_task.h
 * @brief The FreeRTOS task for main.c. Implements the sensor through 
 *        STMicroelectronics's driver (which calls test_i2c_hal.h callbacks) 
 *        and logs accel/gyro/temp data once per second.
 **/

#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "IMU_ISM330DLC.h"
#include "test_i2c_hal.h"

/**
 * @brief FreeRTOS task entry point (matches FreeRTOS's TaskFunction_t: void(*)(void*)).
 *        as required by xTaskCreate(). Initialises the IMU then loops forever, logging 
 *        accel, gyro, and temp once a second. If the sensor does not respond correctly 
 *        on startup, logs the error and yeets itself to the land of deletion instead.  
 */
void test_ism330dlc_task(void *arg);