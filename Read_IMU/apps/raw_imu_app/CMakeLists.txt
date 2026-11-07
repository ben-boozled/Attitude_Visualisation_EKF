set(SOURCE_FILES 
    "main.c"
    "test_i2c_hal.c"
    "test_ism330dlc_task.c"
)

idf_component_register(SRCS ${SOURCE_FILES}
                       INCLUDE_DIRS "."
                       REQUIRES 
                           driver          # ESP-IDF GPIO and I2C driver
                           IMU_ISM330DLC   # ../components/IMU_ISM330DLC - IMU driver
)
