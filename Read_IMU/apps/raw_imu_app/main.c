/** 
 * @file main.c
 * @brief ESP-IDF application used to test I2C communication between an ESP32-C6-Mini-1 
 *        board and STMicroelectronics' ISM330DLC IMU. Directly uses STMicroelectronics' 
 *        driver for the ISM330DLC IMU. Reads the raw IMU data, returning accelerometer,
 *        gyroscope and temperature measurements.
 **/

#include "esp_log.h"

#include "IMU_ISM330DLC.h"
#include "test_i2c_hal.h"
#include "test_ism330dlc_task.h"

static const char *TAG = "raw ism330dlc test";

void app_main(void)
{
    ESP_ERROR_CHECK(test_i2c_hal_init());
    ESP_LOGI(TAG, "I2C master bus initialised.");

    /* Create a task named "ism330dlc_test" that runs the function test_ism330dlc_task
       - Assign stack size of 4096 bytes (default based on ESP IDF's examples + the 
         task footprint itself is small).
       - Assign execution priority level 5 (in FreeRTOS, higher number = higher priority). */
    xTaskCreate(test_ism330dlc_task, "raw ism330dlc test", 4096, NULL, 5, NULL);
}