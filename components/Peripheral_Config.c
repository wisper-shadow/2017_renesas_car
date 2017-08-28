/*
 * Peripheral_Config.c
 *
 *  Created on: 2017Äê8ÔÂ26ÈÕ
 *      Author: Arthur
 */

#include "Peripheral_Config.h"

bool is_alarm = false;

uint8_t uart1_rx = 0xff;
uint8_t uart5_rx = 0xff;

void GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
}

void Timer0A_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10);
    IntMasterEnable();
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void ConfigureUART0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTStdioConfig(0, 115200, 16000000);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTprintf("system clock:%u\n", SysCtlClockGet());
}

void UART0_IntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);
    while(UARTCharsAvail(UART0_BASE))
        UARTCharGetNonBlocking(UART0_BASE);
}

void ConfigureUART1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART1_BASE,  SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void UART1_IntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32Status);
    while(UARTCharsAvail(UART1_BASE))
    {
        uart1_rx = UARTCharGetNonBlocking(UART1_BASE);
        switch(uart1_rx)
        {
        case 0xad:
            UARTCharPut(UART5_BASE, 0xad);
            break;
        case 0xac:
            UARTCharPut(UART5_BASE, 0xac);
            break;
        case 0x01:
            Key_GO();
            break;
        case 0x02:
            Key_BACK();
            break;
        case 0x03:
            Key_LEFT();
            break;
        case 0x04:
            Key_RIGHT();
            break;
        case 0x05:
            Key_STOP();
            break;
        case 0x06:
            Key_LEFT_90();
            break;
        case 0x07:
            Key_RIGHT_90();
            break;
        default:
            break;
        }
    }
}

void ConfigureUART5(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTConfigSetExpClk(UART5_BASE,  SysCtlClockGet(), 57600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
}

void UART5_IntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART5_BASE, true);
    UARTIntClear(UART5_BASE, ui32Status);
    while(UARTCharsAvail(UART5_BASE))
    {
        uart5_rx = UARTCharGetNonBlocking(UART5_BASE);
        if(uart5_rx == 0xef)
            continue;
        if(uart5_rx == 0xe1)
        {
            is_alarm = true;
            continue;
        }
        if(uart5_rx == 0xe0)
        {
            is_alarm = false;
            continue;
        }
        UARTCharPut(UART1_BASE, uart5_rx);
    }

}

void Peripheral_Enable(void)
{
    GPIO_Init();
    Timer0A_Init();
    ConfigureUART0();
    ConfigureUART1();
    ConfigureUART5();
}

void Priority_Set(void)
{
    IntPrioritySet(INT_TIMER0A, 0x00);          //000 0 0000
    IntPrioritySet(INT_UART0, 0xE0);            //111 0 0000
    IntPrioritySet(INT_UART1, 0x20);            //001 0 0000
    IntPrioritySet(INT_UART5, 0x40);            //010 0 0000
}
