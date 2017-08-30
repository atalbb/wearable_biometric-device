/*
 * lis3dh.c
 *
 *  Created on: Aug 29, 2017
 *      Author: Atalville
 */

#include "driverlib.h"
#include "lis3dh.h"


uint8_t r_reg(uint8_t addr)
{
    uint16_t i;

    CS_LOW()
    i = spi_transfer16(0xff | ((addr | 0x80) << 8));
    CS_HIGH()
    return (uint8_t) (i & 0x00FF);
}
void w_reg(uint8_t addr, uint8_t data)
{
    uint16_t i;
    CS_LOW()
    spi_transfer16( (data & 0xFF) | (addr << 8) );
    CS_HIGH()
}
void lis3dh_init(uint8_t dataRate){
    w_reg(LIS3DH_CTRL_REG1,dataRate|0x07);
    // High res & BDU enabled
    //w_reg(LIS3DH_CTRL_REG4, 0x88);

    // DRDY on INT1
    //w_reg(LIS3DH_CTRL_REG3, 0x10);

    // Turn on orientation config
    //writeRegister8(LIS3DH_REG_PL_CFG, 0x40);

    // enable adcs
    //w_reg(LIS3DH_TEMPCFG_REG, 0x80);
}
void lis3dh_setRange(uint8_t range){
    uint8_t r = r_reg(LIS3DH_CTRL_REG4);
    r &= ~(0x30);
    r |= range;
    w_reg(LIS3DH_CTRL_REG4, r);
}
uint8_t lis3dh_getRange(){
    uint8_t r = r_reg(LIS3DH_CTRL_REG4);
    r = (r & 0x30);
    return r;
}
void lis3dh_readRawData(int16_t *x, int16_t *y, int16_t *z){
    int8_t a[2]={0};
    int8_t b[2]={0};
    int8_t c[2]={0};
    uint8_t aux_status = r_reg(LIS3DH_STATUS_AUX_REG);
    uint8_t status = r_reg(LIS3DH_STATUS_REG);
    printf("AUX Status reg:0x%x\r\n",aux_status);
    printf("Status reg:0x%x\r\n",status);
    //while(status != 0x08);
    //printf("New lis3dh data ready\r\n");
    a[0] = r_reg(LIS3DH_OUT_X_L);
    a[1] = r_reg(LIS3DH_OUT_X_H);

    b[0] = r_reg(LIS3DH_OUT_Y_L);
    b[1] = r_reg(LIS3DH_OUT_Y_H);

    c[0] = r_reg(LIS3DH_OUT_Z_L);
    c[1] = r_reg(LIS3DH_OUT_Z_H);

    *x = a[1]<<8 | a[0];
    *y = b[1]<<8 | b[0];
    *z = c[1]<<8 | c[0];
}
void  lis3dh_readNormalizedData(float *x, float *y,  float *z){
    uint8_t range = lis3dh_getRange();
    int16_t a,b,c;
    float divider = 1.0;
    lis3dh_readRawData(&a,&b,&c);
    if(range == LIS3DH_2G){
        divider = 16380.0;
    }else if(range == LIS3DH_4G){
        divider = 8190.0;
    }else if(range == LIS3DH_8G){
        divider = 4096.0;
    }else if(range == LIS3DH_16G){
        divider = 1365.0;
    }
    *x = a/divider;
    *y = b/divider;
    *z = c/divider;
}
