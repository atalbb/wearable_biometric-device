//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include <stdio.h>
#include <string.h>
#include "driverlib.h"
#include "clk.h"
#include "debug.h"
#include "systick.h"
#include "uart.h"

#define BLE_CTS_PORT        GPIO_PORT_P1
#define BLE_CTS_PIN         GPIO_PIN6
#define BLE_RTS_PORT        GPIO_PORT_P1
#define BLE_RTS_PIN         GPIO_PIN7
#define BLE_MOD_PORT         GPIO_PORT_P2
#define BLE_MOD_PIN         GPIO_PIN3


#define SEND_CMD         uartA2_tx_str
#define DISPLAY_RESP    uartA0_tx_str


#define RX_BUFF_SIZE 256
uint8_t g_rx_buff[RX_BUFF_SIZE];
uint8_t g_ms_timeout = 0;
uint32_t g_check_connection_count = 0;
uint32_t g_data_send_interval_count = 0;
//uint8_t g_sample_bluetooth = 0;
uint8_t g_ble_connect_state = 0;
typedef enum{
    BLE_IDLE = 0,
    BLE_CHECK_CONNECTION,
    BLE_CONNECTED,
    BLE_SEND_DATA
}_E_BLE_STATE;
_E_BLE_STATE g_ble_state = BLE_IDLE;
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
void Systick_init(){
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
void SysTick_Handler(){
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
    reset_rx_buffer();
    //DISPLAY_RESP("Sending AT+BLEUARTTX=temp=20 C \r\n");
    SEND_CMD("AT+BLEUARTTX=temp=20 C\\r\\n\r\n");
    systick_delay_ms(50);
    return check_response("OK\r\n");
}
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    clk_init();
    debug_init();
    //systick_init(48,1);
    Systick_init();
    dummy_uart_init();
    ble_pins_init();
    MAP_Interrupt_enableMaster();
    uartA0_tx_str(" Program Started!\r\n");
    while(1){

        if(g_ble_state == BLE_CHECK_CONNECTION){
            reset_rx_buffer();
            //DISPLAY_RESP("Sending AT+GAPGETCONN\r\n");
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
