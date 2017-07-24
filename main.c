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
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
uint8_t state = 0;
uint32_t gIrRedCount = 0;
uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats

void main(){
    uint8_t id = 0xff;
    uint8_t data = 0x7;
    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int i=0;
    int32_t n_brightness;
    float f_temp;
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    clk_init();
    debug_init();
    //systick_init(48,1);
    MAP_Interrupt_enableMaster();
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

    /* Configuring P1.0 as output and P1.1 (switch) as input */
     MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

     /* Configuring P1.1 as an input and enabling interrupts */
     MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
     MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
     MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
     MAP_Interrupt_enableInterrupt(INT_PORT1);

    I2C_Init();  // initialize eUSCI
    g_SMCLKfreq=MAP_CS_getSMCLK();  // get SMCLK value to verify it was set correctly
    g_MCLKfreq=MAP_CS_getMCLK();  // get MCLK value
    printf("MCLK = %d Hz, SMCLK = %d Hz\r\n",g_MCLKfreq,g_SMCLKfreq);
    printf("Program started!\r\n");
    I2C_bus_read(0x57, 0xff, &id, 1);
    printf("id = 0x%x\r\n",id);

    I2C_bus_write(0x57,0xC,&data,1);
    I2C_bus_read(0x57, 0xC, &id, 1);
    printf("id = 0x%x\r\n",id);
    maxim_max30102_reset(); //resets the MAX30102
    //read and clear status register
    maxim_max30102_read_reg(0,&uch_dummy);
    maxim_max30102_init();  //initializes the MAX30102


    n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;

    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    while(1){
        if(state == 1){
            maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
            printf("red=");
            printf("%i", aun_red_buffer[499]);
            printf(", ir=");
            printf("%i", aun_ir_buffer[499]);
            printf(", HR=%i, ", n_heart_rate);
            printf("HRvalid=%i, ", ch_hr_valid);
            n_sp02 -= 4;
            printf("SpO2=%i, ", n_sp02);
            printf("SPO2Valid=%i\n\r", ch_spo2_valid);
            state = 2;
        }else if(state == 2){
            i=0;
            un_min=0x3FFFF;
            un_max=0;

            //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
            for(i=100;i<500;i++)
            {
                aun_red_buffer[i-100]=aun_red_buffer[i];
                aun_ir_buffer[i-100]=aun_ir_buffer[i];

                //update the signal min and max
                if(un_min>aun_red_buffer[i])
                un_min=aun_red_buffer[i];
                if(un_max<aun_red_buffer[i])
                un_max=aun_red_buffer[i];
            }
            gIrRedCount = 400;
            state = 3;
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
                if(un_min>aun_red_buffer[gIrRedCount])
                    un_min=aun_red_buffer[gIrRedCount];    //update signal min
                if(un_max<aun_red_buffer[gIrRedCount])
                    un_max=aun_red_buffer[gIrRedCount];    //update signal max
                if(++gIrRedCount == 500){
                    un_prev_data=aun_red_buffer[gIrRedCount-1];
                    gIrRedCount = 0;
                    state = 1;
                }
            }else if(state == 3){
                un_prev_data=aun_red_buffer[gIrRedCount-1];
                if(++gIrRedCount == 500){
                    gIrRedCount = 0;
                    state = 1;
                }
            }
        }
}
/* GPIO ISR */
void PORT1_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    /* Toggling the output on the LED */
    if(status & GPIO_PIN1)
    {
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        printf("toggle\r\n");
    }

}
