//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include "adxl345.h"
#include "driverlib.h"
#include "clk.h"
#include "debug.h"
#include "msp432_spi.h"
#include "systick.h"
//#define CS_LOW()   GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
//#define CS_HIGH()   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
//uint8_t r_reg(uint8_t addr)
//{
//    uint16_t i;
//
//    CS_LOW()
//    i = spi_transfer16(0xff | ((addr | 0x80) << 8));
//    CS_HIGH()
//    return (uint8_t) (i & 0x00FF);
//}
//void w_reg(uint8_t addr, uint8_t data)
//{
//    uint16_t i;
//    CS_LOW()
//    spi_transfer16( (data & 0xFF) | (addr << 8) );
//    CS_HIGH()
//}
volatile uint32_t g_ticks = 0;
volatile uint8_t g_ms_timeout = 0;

void SysTick_Handler(){
    g_ticks++;
    g_ms_timeout = 1;
}
void main(void)
{

    int i = 0;
    uint8_t readValue = 0;
    int16_t x,y,z;
    float a,b,c;
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    clk_init();
    debug_init();
    spi_init();
    systick_init();
    readValue = r_reg(0x00);
//    /printf();
    MAP_Interrupt_enableMaster();
    printf("LIS3DH Program Started!\r\n");
//    lis3dh_init(LIS3DH_400Hz);
//    readValue = r_reg(0x0f);
    printf("device id = 0x%x\r\n",readValue);
    ADXL345_powerOn();
    ADXL345_setRange(ADXL345_RANGE_16_G);
    ADXL345_setDataRate(ADXL345_DATARATE_400_HZ);
//    //w_reg(0x21,0x85);
//    readValue = r_reg(LIS3DH_CTRL_REG1);
//    printf("CTRL_REG1 = 0x%x\r\n",readValue);
//    lis3dh_setRange(LIS3DH_16G);
//    readValue = lis3dh_getRange();
//    printf("LIS3DH Range = 0x%x\r\n",readValue);
    while(1){
        ADXL345_readAccel(&x, &y, &z);
//        printf("Overrun reg = 0x%x\r\n",r_reg(0x07));
        ADXL345_readNormalizedAccel(&a,&b,&c);
        printf("RAW:x = %d, y = %d, z = %d\r\n",x,y,z);
       printf("Normalized:x = %f, y = %f, z = %f\r\n",a,b,c);
        for(i=0;i<1000000;i++);
    }
}
//float a,b,c;
//void PORT2_IRQHandler(){
//    //aun_red_buffer[gIrRedCount] =
//    uint32_t status;
//
//    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);
//        if(status & GPIO_PIN5){
//            lis3dh_readNormalizedData(&a,&b,&c);
//        }
//}
