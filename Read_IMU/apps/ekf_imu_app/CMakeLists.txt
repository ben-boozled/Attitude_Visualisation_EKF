set(SOURCE_FILES 
    "main.cpp"
    "i2c_hal.cpp"
    "imu_ekf_task.cpp"
)

idf_component_register(SRCS ${SOURCE_FILES}
                       INCLUDE_DIRS "."
                       REQUIRES 
                           driver          # ESP-IDF GPIO and I2C driver
                           esp_timer       # esp_timer_get_time() for dt measurement
                           EKF             # ../../components/EKF - Extended Kalman Filter implementation
                           IMU_ISM330DLC   # ../../components/IMU_ISM330DLC - IMU driver
)
