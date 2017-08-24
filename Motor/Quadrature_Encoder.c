/*
 * Quadrature_Encoder.c
 *
 *  Created on: 2017Äê8ÔÂ23ÈÕ
 *      Author: Arthur
 */

#include "Quadrature_Encoder.h"

/*
 * pin assignment
 *
 *-------------------------
 *  TM4C    |    Encoder
 *-------------------------
 *  PD6     |   PhA0
 *  PD7     |   PhB0
 *  PC5     |   PhA1
 *  PC6     |   PhB1
 *
 *-------------------------
 *
 */

#define MOTOR_NUM 2

#define MOTOR_QEI_PH0_GPIO_PERIPH  SYSCTL_PERIPH_GPIOD
#define MOTOR_QEI_PH0_GPIO_PORT    GPIO_PORTD_BASE
#define MOTOR_QEI_PH0_GPIO_PIN     (GPIO_PIN_6 | GPIO_PIN_7)
#define MOTOR_QEI_PH0_GPIO_CFG_A   GPIO_PD6_PHA0
#define MOTOR_QEI_PH0_GPIO_CFG_B   GPIO_PD7_PHB0
#define MOTOR_QEI_PH0_SWAP_FLAG    QEI_CONFIG_SWAP

#define MOTOR_QEI_PH1_GPIO_PERIPH  SYSCTL_PERIPH_GPIOC
#define MOTOR_QEI_PH1_GPIO_PORT    GPIO_PORTC_BASE
#define MOTOR_QEI_PH1_GPIO_PIN     (GPIO_PIN_5 | GPIO_PIN_6)
#define MOTOR_QEI_PH1_GPIO_CFG_A   GPIO_PC5_PHA1
#define MOTOR_QEI_PH1_GPIO_CFG_B   GPIO_PC6_PHB1
#define MOTOR_QEI_PH1_SWAP_FLAG    QEI_CONFIG_NO_SWAP

//
// frequence to calculate velocity
//
#define MOTOR_QEI_VEL_FREQ      20
#define MOTOR_QEI_VEL_PERIOD    (1.0f / MOTOR_QEI_VEL_FREQ)

motor_qei_data_t qei_data_array[MOTOR_NUM] =
{
        {
                .periph = SYSCTL_PERIPH_QEI0,
                .base = QEI0_BASE,
                .swap_flag = MOTOR_QEI_PH0_SWAP_FLAG,
                .iinterrupt = INT_QEI0,

                .gpio_periph = MOTOR_QEI_PH0_GPIO_PERIPH,
                .gpio_port = MOTOR_QEI_PH0_GPIO_PORT,
                .gpio_pin = MOTOR_QEI_PH0_GPIO_PIN,
                .gpio_cfg_a = MOTOR_QEI_PH0_GPIO_CFG_A,
                .gpio_cfg_b = MOTOR_QEI_PH0_GPIO_CFG_B,

                .dir = 1,
                .velocity = 0,
        },

        {
                .periph = SYSCTL_PERIPH_QEI1,
                .base = QEI1_BASE,
                .swap_flag = MOTOR_QEI_PH1_SWAP_FLAG,
                .iinterrupt = INT_QEI1,

                .gpio_periph = MOTOR_QEI_PH1_GPIO_PERIPH,
                .gpio_port = MOTOR_QEI_PH1_GPIO_PORT,
                .gpio_pin = MOTOR_QEI_PH1_GPIO_PIN,
                .gpio_cfg_a = MOTOR_QEI_PH1_GPIO_CFG_A,
                .gpio_cfg_b = MOTOR_QEI_PH1_GPIO_CFG_B,

                .dir = 1,
                .velocity = 0,
        },
};

void Motor_Init_QEI2(motor_qei_data_t *qei_data)
{
    //
    // Enable the QEI0 peripheral
    //
    SysCtlPeripheralEnable(qei_data->periph);
    //
    // Wait for the QEI0 module to be ready.
    //
    while(!SysCtlPeripheralReady(qei_data->periph));

    //
    // Configure gpio
    //
    SysCtlPeripheralEnable(qei_data->gpio_periph);
    while(!SysCtlPeripheralReady(qei_data->gpio_periph));
    if(qei_data->gpio_port == GPIO_PORTD_BASE && (qei_data->gpio_pin & GPIO_PIN_7))
    {
        //
        // unlock PF7
        //
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x4C4F434B;
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x00;
    }
    GPIOPinConfigure(qei_data->gpio_cfg_a);
    GPIOPinConfigure(qei_data->gpio_cfg_b);
    GPIOPinTypeQEI(qei_data->gpio_port, qei_data->gpio_pin);

    //
    // Configure the quadrature encoder to capture edges on both signals and
    // maintain an absolute position by resetting on index pulses. Using a
    // 1000 line encoder at four edges per line, there are 4000 pulses per
    // revolution; therefore set the maximum position to 3999 as the count
    // is zero based.
    //
    QEIConfigure(qei_data->base, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | qei_data->swap_flag), 0xFFFFFFFF);
    //
    // Enable the quadrature encoder.
    //
    QEIEnable(qei_data->base);

    //
    // Configure qei velocity
    //
    QEIVelocityConfigure(qei_data->base, QEI_VELDIV_1, SysCtlClockGet() / MOTOR_QEI_VEL_FREQ);
    QEIVelocityEnable(qei_data->base);

    QEIIntEnable(qei_data->base, QEI_INTDIR | QEI_INTTIMER);

    IntEnable(qei_data->iinterrupt);
}

void Motor_Init_QEI(void)
{
    uint8_t i = 0;
    for(i = 0; i < sizeof(qei_data_array) / sizeof(qei_data_array[0]); i++)
    {
        Motor_Init_QEI2(qei_data_array + i);
    }
}

void QEI_IRQHandler(motor_side_e side)
{
    uint32_t status = QEIIntStatus(qei_data_array[side].base, true);
    QEIIntClear(qei_data_array[side].base, status);

    if(status & QEI_INTINDEX)
    {
        UARTprintf("%s int_index\n", side == MOTOR_LEFT ? "left" : "right");
    }

    if(status & QEI_INTTIMER)
    {
//        UARTprintf("%s int_timer\n", side == MOTOR_LEFT ? "left" : "right");
    }

    if(status & QEI_INTDIR)
    {
        UARTprintf("%s int_dir\n", side == MOTOR_LEFT ? "left" : "right");
        qei_data_array[side].dir = -qei_data_array[side].dir;
    }
}

void QEI0_IRQHandler(void)
{
    QEI_IRQHandler(MOTOR_LEFT);
}

void QEI1_IRQHandler(void)
{
    QEI_IRQHandler(MOTOR_RIGHT);
}

void ConfigureUART0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTprintf("system clock:%u\n", SysCtlClockGet());
}
