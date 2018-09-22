#include "lis2hh12tr.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include "driver/i2c.h"

#define LIS2HH12_DEVICE (30)    // Device Address for LIS2HH12
#define LIS2HH12_TO_READ (1)      // Number of Bytes Read - Two Bytes Per Axis
#define I2C_MASTER_FREQ_HZ         100000

#define I2C_MASTER_NUM                      I2C_NUM_0
#define WRITE_BIT                           0 /*!< I2C master write */
#define READ_BIT                            1  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */

lis2hh12_handle_t lis2hh12_init(lis2hh12_config_t *lis2hh12_config)
{
    lis2hh12_handle_t lis2hh12_handle = malloc(sizeof(lis2hh12_t));
    lis2hh12_handle->status = (bool)LIS2HH12_OK;
	lis2hh12_handle->error_code = LIS2HH12_NO_ERROR;
	lis2hh12_handle->gains[0] = 1;//0.00376390;		// Original gain 0.00376390 
	lis2hh12_handle->gains[1] = 1;//0.00376009;		// Original gain 0.00376009
	lis2hh12_handle->gains[2] = 1;//0.00349265;		// Original gain 0.00349265
    lis2hh12_handle->device_address = lis2hh12_config->device_address;
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = lis2hh12_config->sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = lis2hh12_config->scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    return lis2hh12_handle;
}

static esp_err_t write_to_i2c(lis2hh12_handle_t lis2hh12_handle, uint8_t _address, uint8_t _val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( lis2hh12_handle->device_address << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write_byte(cmd, LIS2HH12_WHO_AM_I, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, _address, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, _val, ACK_CHECK_EN);
    // i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
static esp_err_t lis2hh12_read_from_i2c(lis2hh12_handle_t lis2hh12_handle, uint8_t address, int num, uint8_t *_buf) 
{     
    // _buf[0] = 10;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis2hh12_handle->device_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lis2hh12_handle->device_address << 1) | READ_BIT, ACK_CHECK_EN);
    int idx = 0;
    while(num) {
        i2c_master_read_byte(cmd, _buf + idx, (num == 1));
        idx++;
        num--;
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // printf("Reach here buf: %d\n", _buf[0]);
    return ret;
}


int scan_i2c()
{
    uint8_t address = 0;
    bool found = false;
    for (address = 0; address < 127; address++)
    {
        printf("check %d\n", address);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( address << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, LIS2HH12_WHO_AM_I, ACK_CHECK_EN);
        // i2c_master_write_byte(cmd, _val, ACK_CHECK_EN);
        // i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            found = true;
            break;
        }
        vTaskDelay(200/portTICK_RATE_MS);
    }
    if (!found) {
        return -1;
    } else
        return address;
}

esp_err_t lis2hh12_power_on(lis2hh12_handle_t lis2hh12_handle)
{
    // uint8_t buf[1];
    uint8_t *buf = malloc(2 * sizeof(uint8_t));
    esp_err_t ret;
    buf[0] = 0;
    printf("buffer: %d\n", buf[0]);
    ret = lis2hh12_read_from_i2c(lis2hh12_handle, LIS2HH12_OUT_X_L, 1, buf);
    printf("buffer: %d\n", buf[0]);
    if (ret != ESP_OK) {
        return ret;
    }
	ret = write_to_i2c(lis2hh12_handle, LIS2HH12_CTRL1, 23);
    if (ret != ESP_OK) {
        return ret;
    }
	ret = write_to_i2c(lis2hh12_handle, LIS2HH12_CTRL3, 128);
	if (ret != ESP_OK) {
        return ret;
    }
    ret = write_to_i2c(lis2hh12_handle, LIS2HH12_ACT_THS, 0);	// Activity/Inactivity detection function disabled     
	if (ret != ESP_OK) {
        return ret;
    }
    ret = write_to_i2c(lis2hh12_handle, LIS2HH12_ACT_DUR, 0);	// Activity/Inactivity detection function disabled	
	if (ret != ESP_OK) {
        return ret;
    }
    ret = write_to_i2c(lis2hh12_handle, LIS2HH12_FIFO_CTRL, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}


esp_err_t lis2hh12_read_accel_3(lis2hh12_handle_t lis2hh12_handle, int *x, int *y, int *z) 
{
    uint8_t _buf[6];
	esp_err_t ret = lis2hh12_read_from_i2c(lis2hh12_handle, 40, 6, _buf);	// Read Accel Data from LIS2HH12
	
	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	*x = (((int)_buf[1]) << 8) | _buf[0];   
	*y = (((int)_buf[3]) << 8) | _buf[2];
	*z = (((int)_buf[5]) << 8) | _buf[4];
    uint8_t *buf = malloc(2 * sizeof(uint8_t));
    buf[0] = 0;
    // printf("buffer: %d\n", buf[0]);
    lis2hh12_read_from_i2c(lis2hh12_handle, LIS2HH12_OUT_X_L, 1, buf);
    printf("buffer: %d\n", buf[0]);
    return ret;
}

/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Variables:  x, y and z          */

esp_err_t lis2hh12_read_accel(lis2hh12_handle_t lis2hh12_handle, int *xyz)
{
	return lis2hh12_read_accel_3(lis2hh12_handle, xyz, xyz + 1, xyz + 2);
}

void lis2hh12_get_gxyz(lis2hh12_handle_t lis2hh12_handle, double *xyz)
{
	int i;
	int xyz_int[3];
	lis2hh12_read_accel(lis2hh12_handle, xyz_int);
	for(i=0; i<3; i++){
		xyz[i] = xyz_int[i] * lis2hh12_handle->gains[i];
	}
}


