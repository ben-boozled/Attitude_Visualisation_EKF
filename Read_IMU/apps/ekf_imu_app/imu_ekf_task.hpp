/** 
 * @file  imu_ekf_task.hpp
 * @brief The FreeRTOS task for main.cpp. Implements the sensor through STMicroelectronics's 
 *        driver (which calls i2c_hal.hpp callbacks); uses EKF to fuse gyroscope and accelerometer
 *        data data.   
 **/

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "IMU_ISM330DLC.h"
#include "i2c_hal.hpp"
#include "EKF.hpp"

// Using namespace to prevent naming conflicts with generic function names
namespace imu_ekf_task {

inline constexpr const char *TAG = "imu_ekf_task";

inline constexpr double pi         = 3.1415926535897932384;
inline constexpr double deg_to_rad = pi / 180.0;
inline constexpr double rad_to_deg = 180.0 / pi;

inline constexpr double    loop_period_s     = 1.0 / 104.0;   // match the 104Hz IMU configuration
inline constexpr uint32_t  loop_period_ms    = 10;            // ~100Hz FreeRTOS task rate (should be >= sensor rate)
inline constexpr uint32_t  log_every_N_loops = 100;           // ~1s at 100Hz; the actual sensor reads ~100 times/sec while this throttles the human-readable log to ~1 time/sec

/**
 * @brief FreeRTOS task entry point (matches FreeRTOS's TaskFunction_t: void(*)(void*)).
 *        as required by xTaskCreate(). Initialises the IMU then loops forever, using the 
 *        EKF to fuse gyroscope and acceleromete datad. If the sensor does not respond 
 *        correctly on startup, logs the error and yeets itself to the land of deletion instead.  
 */
void run(void *arg);

}