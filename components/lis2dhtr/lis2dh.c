#include "lis2dh.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include "driver/i2c.h"

#define I2C_MASTER_FREQ_HZ 100000
#define WRITE_BIT 0       /*!< I2C master write */
#define READ_BIT 1        /*!< I2C master read */
#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

static esp_err_t write_reg_multibyte(lis_handle_t lis_handle, uint8_t _address, uint8_t *_val, uint8_t num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis_handle->device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, _address, ACK_CHECK_EN);
    i2c_master_write(cmd, _val, num, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(lis_handle->port_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t write_reg_byte(lis_handle_t lis_handle, uint8_t _address, uint8_t _val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis_handle->device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, _address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, _val, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(lis_handle->port_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t lis_set_range(uint8_t range)
{
    switch (range)
    {
    case LIS2DH12_RANGE_2GA:
        return 16;
    case LIS2DH12_RANGE_4GA:
        return 8;
    case LIS2DH12_RANGE_8GA:
        return 4;
    case LIS2DH12_RANGE_16GA:
        return 2;
    }
    return 16;
}

lis_handle_t lis_init(lis_config_t *lis_config)
{
    lis_handle_t lis_handle = malloc(sizeof(lis_t));
    lis_handle->device_address = lis_config->device_address;
    lis_handle->port_num = lis_config->port_num;
    lis_handle->range = lis_config->range;
    lis_handle->mg_scale = lis_set_range(lis_config->range);
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = lis_config->sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = lis_config->scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(lis_handle->port_num, &conf);
    esp_err_t ret = i2c_driver_install(lis_handle->port_num, conf.mode, 0, 0, 0);
    if (ret == ESP_FAIL) {
        return NULL;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis_handle->device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x0F, ACK_CHECK_EN); // WHO_AM-I reg
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(lis_handle->port_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return NULL;
    }
    else {
        uint8_t ctrl_reg_values[6] = {0x2F, 0x00, 0x00, lis_handle->range, 0x00, 0x00};
        ret = write_reg_multibyte(lis_handle, 0xA0, ctrl_reg_values, 6);
        if (ret != ESP_OK) {
            return NULL;
        }
    }
    return lis_handle;
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
static esp_err_t lis_read_from_i2c(lis_handle_t lis_handle, uint8_t address, uint8_t *_buf, int num)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis_handle->device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(lis_handle->port_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis_handle->device_address << 1) | READ_BIT, ACK_CHECK_EN);
    int idx = 0;
    while (num)
    {
        i2c_master_read_byte(cmd, _buf + idx, (num == 1));
        idx++;
        num--;
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(lis_handle->port_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t lis_read_accel_xyz(lis_handle_t lis_handle, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t _buf[6];
    esp_err_t ret = lis_read_from_i2c(lis_handle, 0xA8, _buf, 6); // Read Accel Data from LIS
    // Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
    *x = (((int16_t)_buf[1]) << 8) | _buf[0];
    *y = (((int16_t)_buf[3]) << 8) | _buf[2];
    *z = (((int16_t)_buf[5]) << 8) | _buf[4];
    return ret;
}

/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Variables:  x, y and z          */

void lis_mg_scale(lis_handle_t lis_handle, int16_t *x, int16_t *y, int16_t *z)
{
    *x = (int32_t)*x * 1000 / (1024 * lis_handle->mg_scale);
    *y = (int32_t)*y * 1000 / (1024 * lis_handle->mg_scale);
    *z = (int32_t)*z * 1000 / (1024 * lis_handle->mg_scale);
}
