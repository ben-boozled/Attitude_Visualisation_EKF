/** 
 * @file  i2c_hal.hpp
 * @brief I2C bus configuration and HAL glue between main.cpp and STMicroelectronic's 
 *        imu_ism330dlc.h/.c driver.        
 **/

#pragma once

#include <cstring>
#include "driver/i2c.h"
#include "IMU_ISM330DLC.h"

// Using namespace to prevent naming conflicts with generic function names like read(), write() etc.
namespace i2c_hal {

inline constexpr i2c_port_t i2c_port        = I2C_NUM_0;
inline constexpr gpio_num_t sda_pin         = GPIO_NUM_6;   // SDA and SCL GPIOs for ESP32-C6-Mini-1
inline constexpr gpio_num_t scl_pin         = GPIO_NUM_7;
inline constexpr uint32_t   i2c_freq_hz     = 400000;        
inline constexpr uint32_t   i2c_timeout_ms  = 1000;

/** IMPORTANT: STM's ISM330DLC_I2C_ADD_L/_H are 8-bit preshifted addresses 
 *  (0xD5/0xD7 = 0x6A/0x6B << 1) for compatibility on STM devices. Espressif's
 *  i2c_master_*_device() calls use plain 7-bit addresses. Hence, STM's constant
 *  has to be shifted one bit down (1 bit shift to the right) */
inline constexpr uint8_t    imu_addr        = (ISM330DLC_I2C_ADD_L >> 1); // 0x6A because pin SD0 on the IMU is pulled to GND

/**
 * @brief Configures and installs the I2C bus. Call once at startup, before creating
 *        any tasks that uses any other I2C drivers. 
 */
esp_err_t init();

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
int32_t write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

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
int32_t read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/**
 * @brief A delay function that is compatible with STMicroelectronics' 
 *        stmdev_ctx_t standard device context structure. Serves as the mdelay
 *        callback under stmdev_ctx_t. Used to halt execution when the sensor needs
 *        time to power up or reset. The function signature matches that of the
 *        STM driver's stmdev_mdelay_ptr.
 * 
 * @param ms The delay duration in ms.
 */
void delay_ms(uint32_t ms);

}