/** 
 * @file test_i2c_hal.c
 * @brief Implementation of test_i2c_hal.h
 **/

#include "test_i2c_hal.h"

esp_err_t test_i2c_hal_init()
{
    /* Define the configuration parameters for the ESP32-C6-Mini 
       as the master I2C device */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,   // disable intenal pullups as the board has external 5k1 pullup resistors
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ,
    };

    /* Set up the I2C device with the given configuration parameters */
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) return err;

    /* Activate the I2C controller and allocate required memory buffers & 
       interrupt registers */
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0); // I2C master does not require buffer for data
}

int32_t test_i2c_hal_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    /* Set the handle to null since the I2C port and device address are constant */
    (void) handle;   // stops compiler warnings on unused parameters

    /* Prepare the stack buffer; in one transaction, ism330dlc writes the register address 
       byte (byte 1) followed by the data byte (byte 2)*/
    uint8_t buf[1+32];          // 1 byte for address + 32 bytes for data (as a safety margin)
    if(len > sizeof(buf)-1){    // sizeof(buf)-1 to extract number of data bytes
        return -1;              // terminates write with error code before writing to buffer if data size is larger than buffer
    }                           
    buf[0] = reg;               // I2C payload assembly, start with the register addres of the sensor 
    memcpy(&buf[1], bufp, len); // copies data bytes (size len) from bufp to &buf[1]

    /* Sends buf to the ism330dlc over I2C_MASTER_PORT, give up if the transaction
       does not complete in I2C_MASTER_TIMEOUT_MS */
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_PORT, ISM330DLC_I2C_ADDR,
                                               buf, (size_t)len + 1,
                                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    return (ret==ESP_OK)? 0 : -1;   // Translate ESP IDF's error checking to ST's convention
}

int32_t test_i2c_hal_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    /* Set the handle to null since the I2C port and device address are constant */
    (void) handle;

    /* Reads I2C register by (1) writing the sensor address (&reg) with an I2C repeated
       start, then (2) reading len bytes back into bufp; does both in a single transaction */
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_PORT, ISM330DLC_I2C_ADDR,
                                                 &reg, 1, bufp, len,
                                                 pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    return (ret==ESP_OK)? 0 : -1; // Translate ESP IDF's error checking to ST's convention
}

void test_i2c_hal_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));  // converts ms into the equivalent number of FreeRTOS scheduler ticks
}