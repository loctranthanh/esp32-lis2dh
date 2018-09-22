#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lis2hh12tr.h"

#define LED_PIN 23

#define GDO2 25

void app_main(){
    // gpio_config_t io_conf;

    // //interrupt disabled
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // //bit mask of the GDO2 pin
    // io_conf.pin_bit_mask = (1<<GDO2);
    // //set as input mode
    // io_conf.mode = GPIO_MODE_INPUT;
    // //enable pull-up mode
    // io_conf.pull_up_en = 1;
    
    // gpio_config(&io_conf);
    // printf("reach here");
    // xTaskCreate(hello_world_task, "hello_world_task", 2048, NULL, 5, NULL);
    lis2hh12_config_t lis2hh12_config = {
        .sda_pin = 33,
        .scl_pin = 32,
        .device_address = 24,
    };
    lis2hh12_handle_t lis2hh12_handle = lis2hh12_init(&lis2hh12_config);
    // uint8_t i2c_addr = scan_i2c();
    // printf("i2c address: %d\n", i2c_addr);
    esp_err_t check = lis2hh12_power_on(lis2hh12_handle);
    if (check != ESP_OK) {
        printf("Init sensor failed!!!\n");
    }
    while (1)
    {
        // int data[3];
		int x = 0, y = 0, z = 0;
        esp_err_t ret = lis2hh12_read_accel_3(lis2hh12_handle, &x, &y, &z);
        if (ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
        } else if (ret == ESP_OK) {
            printf("Receive data: x: %d, y: %d, z: %d\n", x, y, z);
        } else {
            printf("%s: No ack, sensor not connected...skip...\n", esp_err_to_name(ret));
        }
        vTaskDelay(2000/portTICK_RATE_MS);
    }
    free(lis2hh12_handle);
}

// #include <driver/i2c.h>
// #include <esp_log.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <stdio.h>
// #include "sdkconfig.h"

// #define SDA_PIN 33
// #define SCL_PIN 32

// static char tag[] = "i2cscanner";

// void task_i2cscanner(void *ignore) {
// 	ESP_LOGD(tag, ">> i2cScanner");
// 	i2c_config_t conf;
// 	conf.mode = I2C_MODE_MASTER;
// 	conf.sda_io_num = SDA_PIN;
// 	conf.scl_io_num = SCL_PIN;
// 	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
// 	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
// 	conf.master.clk_speed = 100000;
// 	i2c_param_config(I2C_NUM_0, &conf);

// 	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

// 	int i;
// 	esp_err_t espRc;
// 	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
// 	printf("00:         ");
// 	for (i=3; i< 0x78; i++) {
// 		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// 		i2c_master_start(cmd);
// 		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
// 		i2c_master_stop(cmd);

// 		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
// 		if (i%16 == 0) {
// 			printf("\n%.2x:", i);
// 		}
// 		if (espRc == 0) {
// 			printf(" %.2x", i);
// 		} else {
// 			printf(" --");
// 		}
// 		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
// 		i2c_cmd_link_delete(cmd);
// 	}
// 	printf("\n");
// 	vTaskDelete(NULL);
// }

// void app_main()
// {
//     xTaskCreate(task_i2cscanner, "task_i2cscanner", 2048, NULL, 5, NULL);
// }