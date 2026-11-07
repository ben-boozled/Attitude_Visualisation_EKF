/** 
 * @file  imu_ekf_task.cpp
 * @brief Implementation of imu_ekf_task.hpp        
 **/

#include "imu_ekf_task.hpp"

namespace imu_ekf_task{

/**
 * @brief Helper function that logs and returns false if an ST call reports an error (non-zero)
 * 
 * @param rc Return code from an ST driver call (ST drivers return type int32_t, returning
 *           0 means success while anything else is a failure)
 * @param what Label for the log message
 */ 
bool st_ok(int32_t rc, const char *what)
{
    if(rc != 0){
        ESP_LOGE(TAG, "%s failed (rc=%d). Check wiring / SD0 level / CS level.",
                 what, static_cast<int> (rc));
        return false;
    }
    return true;
}

/**
 * @brief Helper function to initialise IMU
 * 
 * @param ctx The STMicroelectronics' stmdev_ctx_t standard device context structure
 *            for the IMU.
 */ 
bool imu_init(stmdev_ctx_t *ctx)
{
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
    if(!st_ok(ism330dlc_device_id_get(ctx, &whoami), "device_id_get")) return false;
    if(whoami != ISM330DLC_ID) {
        ESP_LOGE(TAG, "WHO_AM_I check failed (got 0x%02X, expected 0x%2X)."
                      "Check wiring / SD0 level / CS level.", whoami, static_cast<unsigned> (ISM330DLC_ID));
        return false;
    }

    /* Perform a power-on software reset on the IMU to force all logic circuits into
       a known state */
    // First, trigger the software reset by writing 1 to the SW_RESET bit
    if(!st_ok(ism330dlc_reset_set(ctx, PROPERTY_ENABLE), "reset_set")) return false;
    
    // Next, check the status of the reset process (bound number of attempts to 20 to prevent hanging)
    uint8_t rst = 1;  // should be cleared to 0 once the device finishes resetting
    for(int attempt = 0; attempt < 20 && rst !=0; attempt++) {
        i2c_hal::delay_ms(1);
        if (!st_ok(ism330dlc_reset_get(ctx, &rst), "reset_get")) return false;
    }

    // Lastly, confirm whether the reset succeeded. If reset !=0, it means the attempt 
    // counter ran out (i.e. reset failed)
    if(rst != 0) {
        ESP_LOGE(TAG, "reset never completed.");
        return false;
    }

     /* Perform sensor configurations */

    // Enable block data update
    // Without this, if a sensor's low and high bytes are read in separate transactions
    // the sensor could update its output between reads, giving a torn value :(
    if(!st_ok(ism330dlc_block_data_update_set(ctx, PROPERTY_ENABLE), "block_data_update_set")) return false;

    // Configure the accelerometer and gyroscope's output data rate (104Hz) and
    // measurement range (+- 4g, += 500 deg/s) 
    if(!st_ok(ism330dlc_xl_data_rate_set(ctx, ISM330DLC_XL_ODR_104Hz), "xl_data_rate_set")) return false;
    if(!st_ok(ism330dlc_xl_full_scale_set(ctx, ISM330DLC_4g), "xl_full_scale_set")) return false;
    if(!st_ok(ism330dlc_gy_data_rate_set(ctx, ISM330DLC_GY_ODR_104Hz), "gy_data_rate_set")) return false;
    if(!st_ok(ism330dlc_gy_full_scale_set(ctx, ISM330DLC_500dps), "gy_full_scale_set")) return false;

    // Select the high performance mode (over the low power mode) for both sensors.
    // Higher current draw but more resilient to noise
    if(!st_ok(ism330dlc_xl_power_mode_set(ctx, ISM330DLC_XL_HIGH_PERFORMANCE), "xl_power_mode_set")) return false;
    if(!st_ok(ism330dlc_gy_power_mode_set(ctx, ISM330DLC_GY_HIGH_PERFORMANCE), "gy_power_mode_set")) return false;

    return true;
}

/**
 * @brief Helper function to read accelerometer data from the IMU
 * 
 * @param ctx The STMicroelectronics' stmdev_ctx_t standard device context structure
 *            for the IMU.
 * @param g The buffer location to store the accelerometer readings (units in g).
 */ 
bool read_accel_g(stmdev_ctx_t *ctx, double g[3])
{
    int16_t raw[3];
    if(ism330dlc_acceleration_raw_get(ctx, raw) != 0) return false;

    /* ISM330DLC_4g was selected in imu_init() -> need to match the ST helper 
       - Convert raw counts to physical units.
       - Divide mg (milli g) by 1000 to get g. */
    for(int i = 0; i < 3; i++) g[i] = static_cast<double> (ism330dlc_from_fs4g_to_mg(raw[i])) / 1000.0;
    return true;
}

/**
 * @brief Helper function to read gyroscope data from the IMU
 * 
 * @param ctx The STMicroelectronics' stmdev_ctx_t standard device context structure
 *            for the IMU.
 * @param dps The buffer location to store the gyroscope readings (rad/s).
 */ 
bool read_gyro_dps(stmdev_ctx_t *ctx, double dps[3])
{
    int16_t raw[3];
    if(ism330dlc_angular_rate_raw_get(ctx, raw) != 0) return false;

    /* ISM330DLC_500dps was selected in imu_init() -> need to match the ST helper 
       - Convert raw counts to physical units.
       - Divide md (milli degrees) by 1000 to get degrees.*/
    for(int i = 0; i < 3; i++) dps[i] = static_cast<double> (ism330dlc_from_fs500dps_to_mdps(raw[i])) / 1000.0;
    return true;
}

void run(void *arg)
{
    stmdev_ctx_t ctx{};
    ctx.write_reg = i2c_hal::write;
    ctx.read_reg = i2c_hal::read;
    ctx.mdelay = i2c_hal::delay_ms;
    ctx.handle = nullptr;

    if(!imu_init(&ctx)){
        vTaskDelete(nullptr); // deletes the FreeRTOS task
        return; 
    }
    ESP_LOGI(TAG, "ISM330DLC initialised okay! (WHO_AM_I confirmed)");

    Imu_ekf::Config cfg{}; // see EKF.hpp for default parameters
    Imu_ekf ekf(cfg);

    // Seed the initial orientation from the first accelerometer reading (the device
    // should be roughly stationary on power-up for this to be a good estimate)
    // Wait ~100ms for the sensor's first data ready flag
    bool seeded = false;
    for(int attempt = 0; attempt < 10 && !seeded; attempt++){
        ism330dlc_status_reg_t status;
        double g[3];
        if(ism330dlc_status_reg_get(&ctx, &status) == 0 && status.xlda && read_accel_g(&ctx, g)){
            ekf.initialise_from_accel(g[0], g[1], g[2]);
            seeded = true;
        } else {
            i2c_hal::delay_ms(10);
        }
    }
    if(!seeded){
        ESP_LOGW(TAG, "No accelerometer sample for initial orientation. Starting from identity quaternion.");
    }

    int64_t last_us = esp_timer_get_time(); // microseconds
    uint32_t loop_count = 0;

    while(true){
        vTaskDelay(pdMS_TO_TICKS(loop_period_ms));

        int64_t now_us = esp_timer_get_time(); // microseconds
        double dt = static_cast<double> (now_us - last_us) / 1.0e6;
        last_us = now_us;
        if(dt <= 0.0) dt = loop_period_s; // guard against clock oddities

        /* Reads the ISM330DLC status_reg into the status struct and remembers whether
           the read succeeded using status*/
        ism330dlc_status_reg_t status;
        if(ism330dlc_status_reg_get(&ctx, &status) == 0){
            double gyro_dps[3];

            /* Reads gyroscope; gda = gyroscope data available */
            if(status.gda && read_gyro_dps(&ctx, gyro_dps)){
                ekf.predict(gyro_dps[0] * deg_to_rad, gyro_dps[1] * deg_to_rad, 
                            gyro_dps[2] * deg_to_rad, dt);
            }

            double accel_g[3];

            /* Reads accelerometer; xlda = accelerometer data available */
            if(status.xlda && read_accel_g(&ctx, accel_g)){
                ekf.update_accel(accel_g[0], accel_g[1], accel_g[2]);
            }
        }

        // Throttles the log to 1 time/sec for readability (instead of following the
        // actual sensor reading loop of ~100 times/sec)
        if(++loop_count >= log_every_N_loops){
            loop_count = 0;
            double roll, pitch, yaw, bias[3];
            ekf.get_euler_rpy(roll, pitch, yaw);
            ekf.get_gyro_bias(bias);
            ESP_LOGI(TAG, "RPY [deg] roll=%.2f pitch=%.2f yaw=%.2f | Bias [dps] %.3f %.3f %.3f",
                     roll*rad_to_deg, pitch*rad_to_deg, yaw*deg_to_rad,
                     bias[0]*rad_to_deg, bias[1]*deg_to_rad, bias[3]*deg_to_rad);
        }

        // Machine parsable line for the Attitude Visualiser project
        // Sends every loop (~100Hz) so the on-screen rotation on the visualiser looks smooth
        // Uses printf (not ESP_LOGI) to avoid the "I (12345) tag:" prefix (if not the visualiser
        // will need to strip the prefix away)
        {
            double q[4];
            ekf.get_quaternion(q);
            printf("QUAT,%.6f,%.6f,%.6f,%.6f\n", q[0], q[1], q[2], q[3]);
        }
    }
}

}
