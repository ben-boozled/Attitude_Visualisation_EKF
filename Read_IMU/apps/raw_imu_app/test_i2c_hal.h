/** 
 * @file test_i2c_hal.h
 * @brief I2C bus configuration and HAL glue between main.c and STMicroelectronic's 
 *        imu_ism330dlc.h/.c driver.        
 **/

#pragma once

#include <stdint.h>
#include <string.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "IMU_ISM330DLC.h"

#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SDA_IO       GPIO_NUM_6     // SDA and SCL GPIOs for ESP32-C6-Mini-1
#define I2C_MASTER_SCL_IO       GPIO_NUM_7   
#define I2C_MASTER_FREQ         400000         // Hz
#define I2C_MASTER_TIMEOUT_MS   1000

/** IMPORTANT: STM's ISM330DLC_I2C_ADD_L/_H are 8-bit preshifted addresses 
 *  (0xD5/0xD7 = 0x6A/0x6B << 1) for compatibility on STM devices. Espressif's
 *  i2c_master_*_device() calls use plain 7-bit addresses. Hence, STM's constant
 *  has to be shifted one bit down (1 bit shift to the right) */
#define ISM330DLC_I2C_ADDR      (ISM330DLC_I2C_ADD_L >> 1)  // 0x6A because pin SD0 on the IMU is pulled to GND

/**
 * @brief Configures and installs the I2C bus. Call once at startup, before creating
 *        any tasks that uses any other I2C drivers. 
 */
esp_err_t test_i2c_hal_init(void);

/**
 * @brief An I2C write function that is compatible with STMicroelectronics' 
 *        stmdev_ctx_t standard device context structure. Serves as the write_reg
 *        callback under stmdev_ctx_t. The function signature matches that of the
 *        STM driver's stmdev_write_ptr.
 * 
 * @param handle The microcontroller's bus configuration structure (e.g. SPI, I2C1, I2C2).
 * @param reg The internal register address of the sensor to write to.
 * @param bufp The prepared data in the microcontroller's RAM to write to the reg.
 * @param len The length or number of data bytes to transfer.
 */
int32_t test_i2c_hal_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

/**
 * @brief An I2C read function that is compatible with STMicroelectronics' 
 *        stmdev_ctx_t standard device context structure. Serves as the read_reg
 *        callback under stmdev_ctx_t. The function signature matches that of the
 *        STM driver's stmdev_read_ptr.
 * 
 * @param handle The microcontroller's bus configuration structure (e.g. SPI, I2C1, I2C2).
 * @param reg The internal register address of the sensor to read from.
 * @param bufp The data buffer to save the bytes the microcontroller reads from the sensor.
 * @param len The length or number of data bytes to transfer.
 */
int32_t test_i2c_hal_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/**
 * @brief A delay function that is compatible with STMicroelectronics' 
 *        stmdev_ctx_t standard device context structure. Serves as the mdelay
 *        callback under stmdev_ctx_t. Used to halt execution when the sensor needs
 *        time to power up or reset. The function signature matches that of the
 *        STM driver's stmdev_mdelay_ptr.
 * 
 * @param ms The delay duration in ms.
 */
void test_i2c_hal_delay_ms(uint32_t ms);