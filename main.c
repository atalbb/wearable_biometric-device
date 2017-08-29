//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include "driverlib.h"
#include "clk.h"
#include "debug.h"
#include "msp432_spi.h"
#define CS_LOW()   GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
#define CS_HIGH()   GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
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
void main(void)
{

    int i = 0;
    uint8_t readValue = 0;
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    clk_init();
    debug_init();
    spi_init();
    MAP_Interrupt_enableMaster();
    printf("LIS3DH Program Started!\r\n");
    while(1){
        readValue = r_reg(0x0f);
        printf("device id = 0x%x\r\n",readValue);
        w_reg(0x21,0x85);
        readValue = r_reg(0x21);
        printf("CTRL_REG1 = 0x%x\r\n",readValue);
        for(i=0;i<50000;i++);
    }
}
//void main(void)
//{
//
//    int i = 0;
//    uint8_t readValue = 0;
//    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
//    clk_init();
//    debug_init();
//    spi_init();
//    MAP_Interrupt_enableMaster();
//    printf("LIS3DH Program Started!\r\n");
//    while(1){
//        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
//        spi_transfer(0x8f);
//        readValue = spi_transfer(0xff);
//        //readValue = r_reg(0x8f);
//        for(i=0;i<5000;i++);
//        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
//        printf("device id = 0x%x\r\n",readValue);
//        for(i=0;i<50000;i++);
//    }
//}
