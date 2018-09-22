#ifndef _LIS2HH12TR_H
#define _LIS2HH12TR_H

#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include "driver/i2c.h"

/*************************** REGISTER MAP ***************************/
#define LIS2HH12_RESERVED0		0x00		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED1		0x01		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED2		0x02		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED3		0x03		// Reserved. Do Not Access. 
#define LIS2HH12_RESERVED4		0x04		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED5		0x05		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED6		0x06		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED7		0x07		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED8		0x08		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED9		0x09		// Reserved. Do Not Access.
#define LIS2HH12_RESERVEDA		0x0A		// Reserved. Do Not Access.
#define LIS2HH12_TEMP_L			0x0B		// Temperature Low Significant Byte (Read)
#define LIS2HH12_TEMP_H			0x0C		// Temperature High Significant Byte (Read)
#define LIS2HH12_RESERVEDE		0x0E		// Reserved. Do Not Access.
#define LIS2HH12_WHO_AM_I		0x0F		// Device ID. (Read)
#define LIS2HH12_ACT_THS		0x1E		// ? (Read/Write)
#define LIS2HH12_ACT_DUR		0x1F		// ? (Read/Write)
#define LIS2HH12_CTRL1			0x20		// Control Register (Read/Write)
#define LIS2HH12_CTRL2			0x21		// Control Register (Read/Write)
#define LIS2HH12_CTRL3			0x22		// Control Register (Read/Write)
#define LIS2HH12_CTRL4			0x23		// Control Register (Read/Write)
#define LIS2HH12_CTRL5			0x24		// Control Register (Read/Write)
#define LIS2HH12_CTRL6			0x25		// Control Register (Read/Write)
#define LIS2HH12_CTRL7			0x26		// Control Register (Read/Write)
#define LIS2HH12_STATUS			0x27		// Status Data Register (Read)
#define LIS2HH12_OUT_X_L		0x28		// X-Axis_Low BYTE (READ)
#define LIS2HH12_OUT_X_H		0x29		// X-Axis_High BYTE (READ)
#define LIS2HH12_OUT_Y_L		0x2A		// X-Axis_Low BYTE (READ)
#define LIS2HH12_OUT_Y_H		0x2B		// X-Axis_High BYTE (READ)
#define LIS2HH12_OUT_Z_L		0x2C		// X-Axis_Low BYTE (READ)
#define LIS2HH12_OUT_Z_H		0x2D		// X-Axis_High BYTE (READ)
#define LIS2HH12_FIFO_CTRL		0x2E		// FIFI Control (Read/Write)
#define LIS2HH12_FIFO_SRC		0x2F		// FIFO ? (Read)
#define LIS2HH12_IG_CFG1		0x30		// Interrupt Generator 1 configuration (Read/Write)
#define LIS2HH12_IG_SRC1		0x31		// Interrupt Generator 1 status Register (Read)
#define LIS2HH12_IG_THS_X1		0x32		// Interrupt generator 1 X Threshold (Read/Write)
#define LIS2HH12_IG_THS_Y1		0x33		// Interrupt Generator 1 Y Threshold (Read/Write)
#define LIS2HH12_IG_THS_Z1		0x34		// Interrupt Generator 1 Z Threshold (Read/Write)
#define LIS2HH12_IG_DUR1		0x35		// Interrupt Generator 1 Duration (Read/Write)
#define LIS2HH12_IG_CFG2		0x36		// Interrupt Generator 2 configuration (Read/Write)
#define LIS2HH12_IG_SRC2		0x37		// Interrupt Generator 2 status Register (Read)
#define LIS2HH12_IG_THS2		0x38		// Interrupt generator 2 Threshold (Read/Write)
#define LIS2HH12_IG_DUR2		0x39		// Interrupt Generator 2 Duration (Read/Write)

#define LIS2HH12_XL_REFERENCE	0x3A		// Reference X Low (Read/Write)
#define LIS2HH12_XH_REFERENCE	0x3B		// Reference X High (Read/Write)
#define LIS2HH12_YL_REFERENCE	0x3C		// Reference Y Low (Read/Write)
#define LIS2HH12_YH_REFERENCE	0x3D		// Reference Y High (Read/Write)
#define LIS2HH12_ZL_REFERENCE	0x3E		// Reference Z Low (Read/Write) 
#define LIS2HH12_ZH_REFERENCE	0x3F		// Reference Z High (Read/Write)


 /************************** INTERRUPT PINS **************************/
#define LIS2HH12_INT1_PIN		0x00		//INT1: 0
#define LIS2HH12_INT2_PIN		0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define LIS2HH12_INT_DATA_READY_BIT		0x07
#define LIS2HH12_INT_SINGLE_TAP_BIT		0x06
#define LIS2HH12_INT_DOUBLE_TAP_BIT		0x05
#define LIS2HH12_INT_ACTIVITY_BIT		0x04
#define LIS2HH12_INT_INACTIVITY_BIT		0x03
#define LIS2HH12_INT_FREE_FALL_BIT		0x02
#define LIS2HH12_INT_WATERMARK_BIT		0x01
#define LIS2HH12_INT_OVERRUNY_BIT		0x00

#define LIS2HH12_DATA_READY				0x07
#define LIS2HH12_SINGLE_TAP				0x06
#define LIS2HH12_DOUBLE_TAP				0x05
#define LIS2HH12_ACTIVITY				0x04
#define LIS2HH12_INACTIVITY				0x03
#define LIS2HH12_FREE_FALL				0x02
#define LIS2HH12_WATERMARK				0x01
#define LIS2HH12_OVERRUNY				0x00


 /****************************** ERRORS ******************************/
#define LIS2HH12_OK			true		// No Error
#define LIS2HH12_ERROR		false		// Error Exists

#define LIS2HH12_NO_ERROR	0		// Initial State
#define LIS2HH12_READ_ERROR	1		// Accelerometer Reading Error
#define LIS2HH12_BAD_ARG    2		// Bad Argument

typedef struct {
    bool status;
    uint8_t error_code;
    double gains[3];
    uint8_t _buff[2] ;		//	2 Bytes Buffer
    uint16_t    device_address;
} lis2hh12_t;

typedef lis2hh12_t* lis2hh12_handle_t;

typedef struct {
    gpio_num_t  sda_pin;
    gpio_num_t  scl_pin;
    uint16_t    device_address;
} lis2hh12_config_t;

lis2hh12_handle_t lis2hh12_init(lis2hh12_config_t *lis2hh12_config);
esp_err_t lis2hh12_power_on(lis2hh12_handle_t lis2hh12_handle);
esp_err_t lis2hh12_read_accel(lis2hh12_handle_t lis2hh12_handle, int *xyz);
esp_err_t lis2hh12_read_accel_3(lis2hh12_handle_t lis2hh12_handle, int *x, int *y, int *z) ;
void lis2hh12_get_gxyz(lis2hh12_handle_t lis2hh12_handle, double *xyz);
int scan_i2c();

#endif