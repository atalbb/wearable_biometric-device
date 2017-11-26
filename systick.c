/*
 * systick.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Atalville
 */
#include "driverlib.h"

extern volatile uint8_t g_ms_timeout;
static uint32_t g_ms_ticks;

void systick_init(){
    SysTick->CTRL = 0;  // disable systick
    SysTick->LOAD = 48000;
    SysTick->VAL = 0; // clear this register
    SysTick->CTRL = 0x00000007;
}
void systick_delay_ms(uint32_t ms){
    while(--ms != 0){
        while(g_ms_timeout == 0);
        g_ms_timeout = 0;
    }
}

//bool TimeOut(unsigned int *timep,unsigned int msec)
//{
//    unsigned int tim;
//    unsigned int diff;
//    bool tout=0;
//
//    tim = tick;
//    while(tim!=tick)
//    {
//        tim = tick;
//    }
//
//    if(tim >= *timep)
//    {
//        diff = (tim - *timep);
//    }
//    else
//    {
//        diff=(0xFFFFFFFF-*timep)+tim;
//    }
//
//    if (msec==0 || diff>=msec)
//    {
//        *timep = tim;
//        tout = 1;
//    }
//    return(tout);
//}
