/** 
 * @file main.cpp
 * @brief ESP-IDF application that filters the ISM330DLC IMU measurements with an extended Kalman
 *        filter. Returns orientation in quaternions.
 **/

#include <stdio.h>
#include "esp_log.h"

#include "IMU_ISM330DLC.h"
#include "imu_ekf_task.hpp"

// restrict TAG visibility to main.cpp only
namespace {
constexpr const char *TAG = "ekf_imu_app";
}

// extern "c" is required so the symbol is emitted as plain app_main(), which is what ESP-IDF expects
// Without extern "c", C++ compilation will cause app_main() to be name mangled into a different form
extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(i2c_hal::init());
    ESP_LOGI(TAG, "I2C master bus initialised.");

    /* Create a task named "ekf imu app" that runs the function imu_ekf_task::run
       - Assign stack size of 8192 bytes (larger than the default 4096 bytes because
         ekf performs matrix computations).
       - Assign execution priority level 5 (in FreeRTOS, higher number = higher priority). */
    xTaskCreate(imu_ekf_task::run, "ekf imu app", 8192, nullptr, 5, nullptr);
}
