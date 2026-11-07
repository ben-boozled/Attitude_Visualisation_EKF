/** 
 * @file test_ism330dlc_task.c
 * @brief Implementation of test_ism330dlc_task.h
 **/

#include "test_ism330dlc_task.h"

static const char *TAG = "ism330dlc test";

void test_ism330dlc_task(void *arg)
{
    (void) arg;  // silences the unused parameter warning

    /* Configure the callback functions for the STM device using STM's context struct */
    stmdev_ctx_t dev_ctx = {
        .write_reg = test_i2c_hal_write,
        .read_reg = test_i2c_hal_read,
        .mdelay = test_i2c_hal_delay_ms,
        .handle = NULL,     // not required since I2C port and address are fixed at compile time in test_ism330dlc_task.c
    };

    /* Checks for two independent failure modes:
       (1) I2C transaction fails (!=0 means a communication error, which will be reported
           by the -1/0 convention as implemented in test_i2c_hal.c and imu_ism330dlc.c). 
           Note that within ism330dlc_device_id_get(), the test_i2c_hal_read() callback 
           will be called.
       (2) The I2C transaction succeeds; ism330dlc_device_id_get() reads the WHO_AMI_I
           register (a fixed, read-only ID byte within the sensor) and stores the ID in the
           buffer at &whoami. However, this ID is the wrong device ID (usually means a
           wrong device, wrong address, or a wiring problem). */
    uint8_t whoami = 0;
    if(ism330dlc_device_id_get(&dev_ctx, &whoami) !=0 || whoami != ISM330DLC_ID){
        ESP_LOGE(TAG, "WHO_AM_I check failed (got 0x%02X, expected 0x%2X)."
                      "Check wiring / SD0 level / CS level.", whoami, ISM330DLC_ID);
        vTaskDelete(NULL);  // deletes the FreeRTOS task
        return;
    }

    /* Perform a power-on software reset on the IMU to force all logic circuits into
       a known state */
    // First, trigger the software reset by writing 1 to the SW_RESET bit
    if(ism330dlc_reset_set(&dev_ctx, PROPERTY_ENABLE) != 0){
        ESP_LOGE(TAG, "reset_set failed");
        vTaskDelete(NULL);
        return;
    }

    // Next, check the status of the reset process
    uint8_t rst = 1;    // should be cleared to 0 once the device finishes resetting
    for(int attempt = 0; attempt < 20 && rst != 0; attempt++){
        test_i2c_hal_delay_ms(1);
        if(ism330dlc_reset_get(&dev_ctx, &rst) != 0){
            ESP_LOGE(TAG, "reset_get failed");
            vTaskDelete(NULL);
            return;
        }
    }

    // Lastly, confirm whether the reset succeeded. If reset !=0, it means the attempt 
    // counter ran out (i.e. reset failed)
    if(rst != 0){
        ESP_LOGE(TAG, "reset never completed");
        vTaskDelete(NULL);
        return;
    }

    /* Perform sensor configurations */

    // Enable block data update
    // Without this, if a sensor's low and high bytes are read in separate transactions
    // the sensor could update its output between reads, giving a torn value :(
    ism330dlc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // Configure the accelerometer and gyroscope's output data rate (104Hz) and
    // measurement range (+- 4g, += 500 deg/s) 
    ism330dlc_xl_data_rate_set(&dev_ctx, ISM330DLC_XL_ODR_104Hz);
    ism330dlc_xl_full_scale_set(&dev_ctx, ISM330DLC_4g);
    ism330dlc_gy_data_rate_set(&dev_ctx, ISM330DLC_GY_ODR_104Hz);
    ism330dlc_gy_full_scale_set(&dev_ctx, ISM330DLC_500dps);
    
    // Select the high performance mode (over the low power mode) for both sensors.
    // Higher current draw but more resilient to noise
    ism330dlc_xl_power_mode_set(&dev_ctx, ISM330DLC_XL_HIGH_PERFORMANCE);
    ism330dlc_gy_power_mode_set(&dev_ctx, ISM330DLC_GY_HIGH_PERFORMANCE);

    // Issue a confirmation log before entering the main loop
    ESP_LOGI(TAG, "ISM330DLC initialised okay! (WHO_AM_I=0x%02X confirmed)", whoami);

    while(1){

        /* Reads the ISM330DLC status_reg into the status struct and remembers whether
           the read succeeded using have_status*/
        ism330dlc_status_reg_t status;
        bool have_status = (ism330dlc_status_reg_get(&dev_ctx, &status) == 0);

        /* Reads accelerometer; xlda = accelerometer data available */
        if(have_status && status.xlda){
            int16_t raw[3];
            if(ism330dlc_acceleration_raw_get(&dev_ctx, raw) == 0){
                /* ISM330DLC_4g was selected above -> need to match the ST helper 
                   - Convert raw counts to physical units.
                   - Divide mg (milli g) by 1000 to get g. */
                float x = ism330dlc_from_fs4g_to_mg(raw[0]) / 1000.0f;
                float y = ism330dlc_from_fs4g_to_mg(raw[1]) / 1000.0f;
                float z = ism330dlc_from_fs4g_to_mg(raw[2]) / 1000.0f;
                ESP_LOGI(TAG, "accel [g]    x=%.3f y=%.3f z=%.3f", x, y, z);
            }
        } else if (have_status){
            ESP_LOGW(TAG, "accel data not ready yet");
        }

        /* Reads gyroscope; gda = gyroscope data available */
        if(have_status && status.gda){
            int16_t raw[3];
            if(ism330dlc_angular_rate_raw_get(&dev_ctx, raw) == 0){
                /* ISM330DLC_500dps was selected above -> need to match the ST helper 
                   - Convert raw counts to physical units.
                   - Divide md (milli degrees) by 1000 to get degrees.*/
                float x = ism330dlc_from_fs500dps_to_mdps(raw[0]) / 1000.0f;
                float y = ism330dlc_from_fs500dps_to_mdps(raw[1]) / 1000.0f;
                float z = ism330dlc_from_fs500dps_to_mdps(raw[2]) / 1000.0f;
                ESP_LOGI(TAG, "gyro [dps]   x=%.3f y=%.3f z=%.3f", x, y, z);
            }
        } else if (have_status){
            ESP_LOGW(TAG, "gyro data not ready yet");
        }

        int16_t raw_temp = 0;
        if(ism330dlc_temperature_raw_get(&dev_ctx, &raw_temp) == 0){
            /* Datasheet states that the temperature has a +- 15 degC offset
               tolerance. Treat this temperature reading as a coarse die measurement,
               not a precise ambient one */
            float temp_c = 25.0f + ((float) raw_temp / 256.0f);  // from the datasheet, the conversion rate is 256 bits per degC
            ESP_LOGI(TAG, "temp [C]     %.1f", temp_c);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // makes the log output once per sec although sensor runs at 104Hz
    }
}