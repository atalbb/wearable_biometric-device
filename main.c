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
#include "algorithm.h"
#include "adc.h"
#include "I2C0.h"
#include "MAX30102.h"


/* Struct Definitions */
typedef struct{
    uint16_t rr;
    int32_t hr;
    int32_t sp02;
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

/* Definitions for MAXREFDES117 */
#define MAX30102_INT_PORT    GPIO_PORT_P4
#define MAX30102_INT_PIN     GPIO_PIN6

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


/* Global variables for MAXREFDES117 */
uint32_t g_aun_ir_buffer[500]; //IR LED sensor data
uint32_t g_aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
uint8_t state = 0;
uint32_t gIrRedCount = 0;


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
static uint16_t calculate_RR(volatile uint16_t *samples){
    int16_t threshold= 0;
    int16_t peaks = 0;
    uint32_t avg = rr_find_mean(samples);
    rr_diff_from_mean(samples,g_rr_temp_buff,avg);
    rr_four_pt_MA(g_rr_temp_buff);
    rr_diff_btw_4pt_MA(g_rr_temp_buff);
    rr_two_pt_MA(g_rr_temp_buff);
    rr_hamming_window(g_rr_temp_buff);
    threshold = rr_threshold_calc(g_rr_temp_buff);
    peaks= rr_myPeakCounter(g_rr_temp_buff, RR_BUF_SIZE-HAM_SIZE,threshold);
    printf("Peaks = %d, ",peaks);
    return (60/RR_INITIAL_FRAME_TIME_S) * peaks;
}

/* Local Functions for BLE */
static void reset_rx_buffer(){
    memset(g_rx_buff,0,RX_BUFF_SIZE);
    set_UARTA2_rx_ptr(0);
}
static void ble_pins_init(){
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
static void dummy_uart_init(){
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
static uint8_t check_response(char *resp){
    char *ret;
    ret = strstr(g_rx_buff,resp);
    if(!ret){
        return 0;
    }
    return 1;
}
static uint8_t ble_send_data(){
    static char data[100];
    //static char cmd[50]="AT+BLEUARTTX=";
    static uint32_t i =0;
    sprintf(data, "%d) RR = %d bpm, HR = %d, SP02 = %d\\r\\n\r\n",++i,g_sensor_data.rr,g_sensor_data.hr,g_sensor_data.sp02);
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
/* Local Fuction Definitions for MAXREFDES117 */
void maxrefdes117_init(){
    uint8_t id = 0xff;
    uint8_t uch_dummy;

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
    maxrefdes117_init();
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

        if(state == 1){
            maxim_heart_rate_and_oxygen_saturation(g_aun_ir_buffer, HR_SP02_BUF_SIZE, g_aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
            g_sensor_data.hr = n_heart_rate;
            g_sensor_data.sp02 = n_sp02;
            printf("HR=%i, ", n_heart_rate);
            printf("HRvalid=%i, ", ch_hr_valid);
            n_sp02 -= 4;
            printf("SpO2=%i, ", n_sp02);
            printf("SPO2Valid=%i\n\r", ch_spo2_valid);

            for(i=HR_SP02_STABLE_BUF_SIZE;i<HR_SP02_BUF_SIZE;i++)
            {
                g_aun_red_buffer[i-HR_SP02_STABLE_BUF_SIZE]=g_aun_red_buffer[i];
                g_aun_ir_buffer[i-HR_SP02_STABLE_BUF_SIZE]=g_aun_ir_buffer[i];
            }
            gIrRedCount = HR_SP02_BUF_SIZE - HR_SP02_STABLE_BUF_SIZE;
            state = 2;
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
void PORT4_IRQHandler(){
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(MAX30102_INT_PORT);
    MAP_GPIO_clearInterruptFlag(MAX30102_INT_PORT, status);
        if(status & MAX30102_INT_PIN){
            maxim_max30102_read_fifo(&g_aun_red_buffer[gIrRedCount], &g_aun_ir_buffer[gIrRedCount]);  //read from MAX30102 FIFO
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
