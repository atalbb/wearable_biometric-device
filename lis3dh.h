/*
 * lis3dh.h
 *
 *  Created on: Aug 29, 2017
 *      Author: Atalville
 */

#ifndef LIS3DH_H_
#define LIS3DH_H_
#include "driverlib.h"

#define CS_LOW()   GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
#define CS_HIGH()   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);

#define LIS3DH_1Hz      0x10
#define LIS3DH_10Hz      0x20
#define LIS3DH_25Hz      0x30
#define LIS3DH_50Hz      0x40
#define LIS3DH_100Hz      0x50
#define LIS3DH_200Hz      0x60
#define LIS3DH_400Hz      0x70

#define LIS3DH_2G       0x00
#define LIS3DH_4G       0x10
#define LIS3DH_8G       0x20
#define LIS3DH_16G       0x30

#define LIS3DH_STATUS_AUX_REG  0x07
#define LIS3DH_TEMPCFG_REG     0x1f
#define LIS3DH_CTRL_REG1       0x20
#define LIS3DH_CTRL_REG2       0x21
#define LIS3DH_CTRL_REG3       0x22
#define LIS3DH_CTRL_REG4       0x23
#define LIS3DH_STATUS_REG  0x26
#define LIS3DH_OUT_X_L         0x28
#define LIS3DH_OUT_X_H         0x29
#define LIS3DH_OUT_Y_L         0x2A
#define LIS3DH_OUT_Y_H         0x2B
#define LIS3DH_OUT_Z_L         0x2C
#define LIS3DH_OUT_Z_H         0x2D





uint8_t r_reg(uint8_t addr);
void w_reg(uint8_t addr, uint8_t data);
void lis3dh_init(uint8_t dataRate);
void lis3dh_setRange(uint8_t range);
uint8_t lis3dh_getRange();
void lis3dh_readRawData(int16_t *x, int16_t *y, int16_t *z);
void  lis3dh_readNormalizedData(float *x, float *y,  float *z);

#endif /* LIS3DH_H_ */
