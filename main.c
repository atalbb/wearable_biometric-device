//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include <stdio.h>
#include "driverlib.h"
#include "clk.h"
#include "debug.h"
#include "systick.h"
#include "I2C0.h"
#include "MAX30102.h"
#include "algorithm.h"

#define MAX30102_INT_PORT    GPIO_PORT_P4
#define MAX30102_INT_PIN     GPIO_PIN6

uint32_t g_SMCLKfreq;
uint32_t g_MCLKfreq;
uint32_t aun_ir_buffer[500]; //IR LED sensor data
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
uint8_t state = 0;
uint32_t gIrRedCount = 0;

void main(){
    uint8_t id = 0xff;
    uint8_t data = 0x7;
    int i=0;
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    clk_init();
    debug_init();
    /* Select Port 6 for I2C - Set Pin 4, 5 to input Primary Module Function,
     *   (UCB1SIMO/UCB1SDA, UCB1SOMI/UCB1SCL).
     */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
            GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsInputPinWithPullUpResistor(MAX30102_INT_PORT, MAX30102_INT_PIN);
    GPIO_interruptEdgeSelect(MAX30102_INT_PORT, MAX30102_INT_PIN,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(MAX30102_INT_PORT, MAX30102_INT_PIN);
    MAP_GPIO_enableInterrupt(MAX30102_INT_PORT, MAX30102_INT_PIN);
    MAP_Interrupt_enableInterrupt(INT_PORT4);

    I2C_Init();  // initialize eUSCI
    printf("Program started!\r\n");
    I2C_bus_read(0x57, 0xff, &id, 1);
    printf("id = 0x%x\r\n",id);

    maxim_max30102_reset(); //resets the MAX30102
    //read and clear status register
    maxim_max30102_read_reg(0,&uch_dummy);
    maxim_max30102_init();  //initializes the MAX30102
    MAP_Interrupt_enableMaster();
    while(1){
        if(state == 1){
            maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, HR_SP02_BUF_SIZE, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
            printf("HR=%i, ", n_heart_rate);
            printf("HRvalid=%i, ", ch_hr_valid);
            n_sp02 -= 4;
            printf("SpO2=%i, ", n_sp02);
            printf("SPO2Valid=%i\n\r", ch_spo2_valid);

            for(i=HR_SP02_STABLE_BUF_SIZE;i<HR_SP02_BUF_SIZE;i++)
            {
                aun_red_buffer[i-HR_SP02_STABLE_BUF_SIZE]=aun_red_buffer[i];
                aun_ir_buffer[i-HR_SP02_STABLE_BUF_SIZE]=aun_ir_buffer[i];
            }
            gIrRedCount = HR_SP02_BUF_SIZE - HR_SP02_STABLE_BUF_SIZE;
            state = 2;
        }
    }
}
void PORT4_IRQHandler(){
    //aun_red_buffer[gIrRedCount] =
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(MAX30102_INT_PORT);
    MAP_GPIO_clearInterruptFlag(MAX30102_INT_PORT, status);
        if(status & MAX30102_INT_PIN){
            maxim_max30102_read_fifo(&aun_red_buffer[gIrRedCount], &aun_ir_buffer[gIrRedCount]);  //read from MAX30102 FIFO
            if(state == 0){
                if(++gIrRedCount == HR_SP02_BUF_SIZE){
                    gIrRedCount = 0;
                    state = 1;
                }
            }else if(state == 2){
                if(++gIrRedCount == HR_SP02_BUF_SIZE){
                    gIrRedCount = 0;
                    state = 1;
                }
            }
        }
}

