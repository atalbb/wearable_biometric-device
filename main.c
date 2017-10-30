//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

/* Standard Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string.h>

/* User Includes */
#include "driverlib.h"
#include "clk.h"
#include "debug.h"
#include "systick.h"
#include "uart.h"
#include "RRAlgorithm.h"
#include "adc.h"


/* Struct Definitions */
typedef struct{
    uint16_t rr;
    uint8_t hr;
    uint8_t sp02;
}_S_SENSOR_DATA;
_S_SENSOR_DATA g_sensor_data;

/* Definitions for Respiratory Rate */
#define CPU_CLK_MHZ          48
#define ADC_SAMPLING_1MS     RR_SAMPLE_TIME_MS

/* Definitions for BLE */
#define RX_BUFF_SIZE        256
#define BLE_CTS_PORT        GPIO_PORT_P1
#define BLE_CTS_PIN         GPIO_PIN6
#define BLE_RTS_PORT        GPIO_PORT_P1
#define BLE_RTS_PIN         GPIO_PIN7
#define BLE_MOD_PORT         GPIO_PORT_P2
#define BLE_MOD_PIN         GPIO_PIN3
#define SEND_CMD         uartA2_tx_str
#define DISPLAY_RESP    uartA0_tx_str

/* Global Variables for Respiratory Rate */
volatile uint8_t  g_adc_state = 0;
uint32_t SMCLKfreq;
uint32_t MCLKfreq;
uint32_t g_adcSamplingPeriod = ADC_SAMPLING_1MS;
volatile _E_RR_STATE g_rr_state = RR_INITIAL;
volatile uint16_t g_rr_buff[RR_BUF_SIZE]={0};
volatile int16_t g_rr_temp_buff[RR_BUF_SIZE]={0};
volatile uint16_t g_rr_sample_count = 0;
volatile uint8_t g_rr_cal_signal = 0;
volatile uint32_t g_curADCResult = 0;

/* Global Variables for BLE */
volatile uint8_t g_rx_buff[RX_BUFF_SIZE];
volatile uint8_t g_ms_timeout = 0;
volatile uint32_t g_check_connection_count = 0;
volatile uint32_t g_data_send_interval_count = 0;
volatile uint8_t g_ble_connect_state = 0;
typedef enum{
    BLE_IDLE = 0,
    BLE_CHECK_CONNECTION,
    BLE_CONNECTED,
    BLE_SEND_DATA
}_E_BLE_STATE;
volatile _E_BLE_STATE g_ble_state = BLE_IDLE;

/* Local Functions for Respiratory Rate */
static uint16_t calculate_RR(uint16_t *samples){
    int16_t threshold= 0;
    int16_t peaks = 0;
    uint32_t avg = rr_find_mean(samples);
    diff_from_mean(samples,g_rr_temp_buff,avg);
    four_pt_MA(g_rr_temp_buff);
    diff_btw_4pt_MA(g_rr_temp_buff);
    two_pt_MA(g_rr_temp_buff);
    hamming_window(g_rr_temp_buff);
    threshold = threshold_calc(g_rr_temp_buff);
    peaks= myPeakCounter(g_rr_temp_buff, RR_BUF_SIZE-HAM_SIZE,threshold);
    printf("Peaks = %d, ",peaks);
    return (60/RR_INITIAL_FRAME_TIME_S) * peaks;
}

/* Local Functions for BLE */
void reset_rx_buffer(){
    memset(g_rx_buff,0,RX_BUFF_SIZE);
    set_UARTA2_rx_ptr(0);
}
void ble_pins_init(){
const eUSCI_UART_Config uartAConfig = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
            104,                                      // BRDIV = 26
            0,                                       // UCxBRF = 0
            0,                                       // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                  // No Parity
            EUSCI_A_UART_LSB_FIRST,                  // MSB First
            EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
            EUSCI_A_UART_MODE,                       // UART mode
            EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Mode
            };
//    MAP_GPIO_setAsOutputPin(BLE_CTS_PORT, BLE_CTS_PIN);
//    MAP_GPIO_setAsOutputPin(BLE_MOD_PORT, BLE_MOD_PIN);
//    MAP_GPIO_setAsInputPin(BLE_CTS_PORT, BLE_CTS_PIN);
    /* UART2(115200 bps baudrate) BLE UART Module initialization*/
    uartA2_init(&uartAConfig,g_rx_buff,RX_BUFF_SIZE);
}
void dummy_uart_init(){
    const eUSCI_UART_Config uartAConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
                104,                                      // BRDIV = 26
                0,                                       // UCxBRF = 0
                0,                                       // UCxBRS = 0
                EUSCI_A_UART_NO_PARITY,                  // No Parity
                EUSCI_A_UART_LSB_FIRST,                  // MSB First
                EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
                EUSCI_A_UART_MODE,                       // UART mode
                EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Mode
                };

    /* UARTA0(115.2 kbps baudrate) Debug Module initialization*/
    uartA0_init(&uartAConfig,0,0);
}
uint8_t check_response(char *resp){
    char *ret;
    ret = strstr(g_rx_buff,resp);
    if(!ret){
        return 0;
    }
    return 1;
}
uint8_t ble_send_data(){
    static char data[50];
    static char cmd[50]="AT+BLEUARTTX=";
    sprintf(data, "RR = %d bpm\\r\\n\r\n",g_sensor_data.rr);
    //strcat(cmd,data);
    reset_rx_buffer();
    //DISPLAY_RESP(cmd);
    //DISPLAY_RESP("\r\n");
    //DISPLAY_RESP("Sending AT+BLEUARTTX=temp=20 C \r\n");
    SEND_CMD("AT+BLEUARTTX=");
    SEND_CMD(data);
    systick_delay_ms(50);
    return check_response("OK\r\n");
}

void SysTick_Handler(){
    static uint32_t i = 0;

    g_ms_timeout = 1;
    if(g_ble_state == BLE_IDLE){
        if(++g_check_connection_count == 1000){
            g_ble_state = BLE_CHECK_CONNECTION;
            g_check_connection_count = 0;
        }
    }else if(g_ble_state == BLE_CONNECTED){
        if(++g_data_send_interval_count == 1000){
            g_ble_state = BLE_SEND_DATA;
            g_data_send_interval_count = 0;
        }
    }


    if(g_adc_state == 2){
        if(++i >= g_adcSamplingPeriod){
            i = 0;
            MAP_ADC14_toggleConversionTrigger();
            P1->OUT ^= 0x01;
            g_adc_state = 0;
        }
    }
}

/* On board LED initialization Function
 * Blinking LED is used to verify sampling rate
 * */
static void led_init(){
    /* Selecting P1.0 as output (LED). */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
    GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
}
void main(void)
{
    uint16_t respiratory_rate = 0;
    uint32_t i = 0;
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    clk_init();
    debug_init();
    systick_init();
    adc_init();
    dummy_uart_init();
    ble_pins_init();
    led_init();
    MAP_Interrupt_enableMaster();
    uartA0_tx_str(" Program Started!\r\n");
    while(1){


        if(g_adc_state == 1){
            if(g_rr_cal_signal == 1){ // calculate Respiratory Rate
                g_rr_cal_signal = 0;
                /* Have to calculate RR*/
                g_sensor_data.rr = respiratory_rate = calculate_RR(g_rr_buff);
                printf("Br is %d\r\n",respiratory_rate);
                //dumping the first X sets of samples and shift the last RR_BUF_SIZE-X sets of samples to the top
                for(i=RR_STABLE_BUF_SIZE;i<RR_BUF_SIZE;i++){
                    g_rr_buff[i-RR_STABLE_BUF_SIZE]=g_rr_buff[i];
                }
            }
            g_adc_state = 2;
        }


        if(g_ble_state == BLE_CHECK_CONNECTION){
            reset_rx_buffer();
            SEND_CMD("AT+GAPGETCONN\r\n");
            systick_delay_ms(50);
            if(check_response("0\r\nOK\r\n")==1){
                DISPLAY_RESP("No BLE Connection Found!\r\n");
                g_ble_state = BLE_IDLE;
            }else if(check_response("1\r\nOK\r\n")==1){
                DISPLAY_RESP("BLE Connected\r\n");
                g_ble_state = BLE_CONNECTED;

            }else{
                g_ble_state = BLE_IDLE;
                DISPLAY_RESP("OK not found for AT+GAPGETCONN\r\n");
            }

        }else if(g_ble_state == BLE_SEND_DATA){
            if(ble_send_data()){
                printf("BLE Data Sent Successfully!\r\n");
                g_ble_state = BLE_CONNECTED;
            }else{
                printf("BLE Data Send Fail!\r\n");
                g_ble_state = BLE_CHECK_CONNECTION;
            }

        }
    }
}
void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);
    if (ADC_INT0 & status)
    {
        g_curADCResult = g_rr_buff[g_rr_sample_count++]= MAP_ADC14_getResult(ADC_MEM0);
        g_adc_state = 1;
        switch(g_rr_state){
            case RR_INITIAL:
                if(g_rr_sample_count == RR_BUF_SIZE){
                    g_rr_sample_count = RR_BUF_SIZE - RR_STABLE_BUF_SIZE;
                    g_rr_state = RR_STABLE;
                    g_rr_cal_signal = 1;
                }
                break;
            case RR_STABLE:
                if(g_rr_sample_count == RR_BUF_SIZE){
                    g_rr_sample_count = RR_BUF_SIZE - RR_STABLE_BUF_SIZE;
                    g_rr_cal_signal = 1;
                }
                break;

            default:

                break;

        }
    }
}
