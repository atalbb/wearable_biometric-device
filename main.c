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
void main(void)
{
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
    //read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(GPIO_getInputPinValue(MAX30102_INT_PORT,MAX30102_INT_PIN)==1);   //wait until the interrupt pin asserts

        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO

        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max
        printf("red=");
        printf("%i", aun_red_buffer[i]);
        printf(", ir=");
        printf("%i\n\r", aun_ir_buffer[i]);
    }
    un_prev_data=aun_red_buffer[i];


    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while(1)
    {
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

        //take 100 sets of samples before calculating the heart rate.
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(GPIO_getInputPinValue(MAX30102_INT_PORT,MAX30102_INT_PIN)==1);   //wait until the interrupt pin asserts
            maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

//            if(aun_red_buffer[i]>un_prev_data)
//            {
//                f_temp=aun_red_buffer[i]-un_prev_data;
//                f_temp/=(un_max-un_min);
//                f_temp*=MAX_BRIGHTNESS;
//                n_brightness-=(int)f_temp;
//                if(n_brightness<0)
//                    n_brightness=0;
//            }
//            else
//            {
//                f_temp=un_prev_data-aun_red_buffer[i];
//                f_temp/=(un_max-un_min);
//                f_temp*=MAX_BRIGHTNESS;
//                n_brightness+=(int)f_temp;
//                if(n_brightness>MAX_BRIGHTNESS)
//                    n_brightness=MAX_BRIGHTNESS;
//            }
#if defined(TARGET_KL25Z) || defined(TARGET_MAX32600MBED)
            led.write(1-(float)n_brightness/256);
#endif
            //send samples and calculation result to terminal program through UART
            printf("red=");
            printf("%i", aun_red_buffer[i]);
            printf(", ir=");
            printf("%i", aun_ir_buffer[i]);
            printf(", HR=%i, ", n_heart_rate);
            printf("HRvalid=%i, ", ch_hr_valid);
            printf("SpO2=%i, ", n_sp02);
            printf("SPO2Valid=%i\n\r", ch_spo2_valid);
        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
    }
}
