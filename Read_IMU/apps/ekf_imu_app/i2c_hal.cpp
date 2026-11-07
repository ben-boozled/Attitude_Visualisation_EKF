/** 
 * @file i2c_hal.cpp
 * @brief Implementation of i2c_hal.hpp
 **/

#include "i2c_hal.hpp"

namespace i2c_hal {

esp_err_t init()
{
    /* Define the configuration parameters for the ESP32-C6-Mini 
       as the master I2C device */
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = i2c_freq_hz;

    /* Set up the I2C device with the given configuration parameters */
    esp_err_t err = i2c_param_config(i2c_port, &conf);
    if(err != ESP_OK) return err;

    /* Activate the I2C controller and allocate required memory buffers & 
       interrupt registers */
    return i2c_driver_install(i2c_port, conf.mode, 0, 0, 0); // I2C master does not require buffer for data
}

int32_t write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    /* Prepare the stack buffer; in one transaction, ism330dlc writes the register address 
       byte (byte 1) followed by the data byte (byte 2)*/
    uint8_t buf[1 + 32];                    // 1 byte for address + 32 bytes for data (as a safety margin)
    if(len > sizeof(buf) - 1) return -1;    // sizeof(buf)-1 to extract number of data bytes
    
    buf[0] = reg;                           // I2C payload assembly, start with the register addres of the sensor 
    std::memcpy(&buf[1], bufp, len);        // copies data bytes (size len) from bufp to &buf[1]

    /* Sends buf to the ism330dlc over I2C_MASTER_PORT, give up if the transaction
       does not complete in i2c_timeout_ms */
    esp_err_t ret = i2c_master_write_to_device(i2c_port, imu_addr, buf, 
                                               static_cast<size_t> (len) + 1, 
                                               pdMS_TO_TICKS(i2c_timeout_ms));

    return (ret == ESP_OK) ? 0 : -1;    // Translate ESP IDF's error checking to ST's convention
}

int32_t read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    /* Reads I2C register by (1) writing the sensor address (&reg) with an I2C repeated
       start, then (2) reading len bytes back into bufp; does both in a single transaction */
    esp_err_t ret = i2c_master_write_read_device(i2c_port, imu_addr, &reg, 1, bufp, 
                                                 len, pdMS_TO_TICKS(i2c_timeout_ms));
                                                
    return (ret == ESP_OK) ? 0 : -1;    // Translate ESP IDF's error checking to ST's convention
}

void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));   // converts ms into the equivalent number of FreeRTOS scheduler ticks
}

}