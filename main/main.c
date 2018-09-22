#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "lis2dh.h"

void app_main()
{
    lis_config_t lis_config = {
        .sda_pin = 33,
        .scl_pin = 32,
        .device_address = 24,
        .range = LIS2DH12_RANGE_2GA,
        .port_num = I2C_NUM_1,
    };
    lis_handle_t lis_handle = lis_init(&lis_config);
    if (lis_handle == NULL) {
        printf("Init failed: \n");
        return;
    }
    printf("Complete initial!!!\n");
    while (1)
    {
        int16_t x = 0, y = 0, z = 0;
        esp_err_t ret = lis_read_accel_xyz(lis_handle, &x, &y, &z);
        lis_mg_scale(lis_handle, &x, &y, &z);
        if (ret == ESP_ERR_TIMEOUT) {
            printf("I2C timeout\n");
        }
        else if (ret == ESP_OK) {
            printf("Receive data: x: %d, y: %d, z: %d\n", x, y, z);
        }
        else {
            printf("%s: No ack, sensor not connected...skip...\n", esp_err_to_name(ret));
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    free(lis_handle);
}