/*
 * Motor.c
 *
 *  Created on: 2017Äê8ÔÂ22ÈÕ
 *      Author: Arthur
 */

#include "Motor.h"

/*
 * pin assignment
 *
 *-------------------------
 *  TM4C    |    L298N
 *-------------------------
 *  PB6     |   IN1
 *  PB7     |   IN2
 *  PB4     |   IN3
 *  PB5     |   IN4
 *-------------------------
 *
 */

#define MOTOR_PWM_PERIOD    400

#define MOTOR_PWM_PERIPH    SYSCTL_PERIPH_PWM0
#define MOTOR_PWM_BASE      PWM0_BASE
#define MOTOR_PWM_GEN_0     PWM_GEN_0
#define MOTOR_PWM_GEN_1     PWM_GEN_1

#define MOTOR_PWM_OUT_LEFT_FRONT    PWM_OUT_0
#define MOTOR_PWM_OUT_LEFT_BACK     PWM_OUT_1
#define MOTOR_PWM_OUT_RIGHT_FRONT   PWM_OUT_2
#define MOTOR_PWM_OUT_RIGHT_BACK    PWM_OUT_3

#define MOTOR_PWM_GPIO_PERIPH       SYSCTL_PERIPH_GPIOB
#define MOTOR_PWM_GPIO_PORT         GPIO_PORTB_BASE
#define MOTOR_PWM_GPIO_PIN_CFG0     GPIO_PB6_M0PWM0
#define MOTOR_PWM_GPIO_PIN_CFG1     GPIO_PB7_M0PWM1
#define MOTOR_PWM_GPIO_PIN_CFG2     GPIO_PB4_M0PWM2
#define MOTOR_PWM_GPIO_PIN_CFG3     GPIO_PB5_M0PWM3
#define MOTOR_PWM_GPIO_PIN_NUM      (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5)
#define MOTOR_PWM_OUT_BIT           (PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT)

void Motor_PWM_Init(void)
{
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
        //
        // Enable the PWM0 peripheral
        //
        SysCtlPeripheralEnable(MOTOR_PWM_PERIPH);
        //
        // Wait for the PWM0 module to be ready.
        //
        while (!SysCtlPeripheralReady(MOTOR_PWM_PERIPH));
        //
        // Configure gpio
        //
        SysCtlPeripheralEnable(MOTOR_PWM_GPIO_PERIPH);
        GPIOPinConfigure(MOTOR_PWM_GPIO_PIN_CFG0);
        GPIOPinConfigure(MOTOR_PWM_GPIO_PIN_CFG1);
        GPIOPinConfigure(MOTOR_PWM_GPIO_PIN_CFG2);
        GPIOPinConfigure(MOTOR_PWM_GPIO_PIN_CFG3);
        GPIOPinTypePWM(MOTOR_PWM_GPIO_PORT, MOTOR_PWM_GPIO_PIN_NUM);
        //
        // Configure the PWM generator for count down mode with immediate updates
        // to the parameters.
        //
        PWMGenConfigure(MOTOR_PWM_BASE, MOTOR_PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenConfigure(MOTOR_PWM_BASE, MOTOR_PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        //
        // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
        // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
        // Use this value to set the period.
        //
        PWMGenPeriodSet(MOTOR_PWM_BASE, MOTOR_PWM_GEN_0, MOTOR_PWM_PERIOD);
        PWMGenPeriodSet(MOTOR_PWM_BASE, MOTOR_PWM_GEN_1, MOTOR_PWM_PERIOD);

        Motor_Set_Throttle(MOTOR_LEFT, 0);
        Motor_Set_Throttle(MOTOR_RIGHT, 0);

        //
        // Start the timers in generator 0.
        //
        PWMGenEnable(MOTOR_PWM_BASE, MOTOR_PWM_GEN_0);
        PWMGenEnable(MOTOR_PWM_BASE, MOTOR_PWM_GEN_1);
        //
        // Enable the outputs.
        //
        PWMOutputState(MOTOR_PWM_BASE, MOTOR_PWM_OUT_BIT, true);
}

void Motor_Set_Throttle(motor_side_e side, int speed)
{
    int32_t pulse_width = speed * MOTOR_PWM_PERIOD / 100;

    if(pulse_width > 0)
        pulse_width = pulse_width;
    if(pulse_width < 0)
        pulse_width = pulse_width + MOTOR_PWM_PERIOD;
    if(pulse_width == 0)
        pulse_width = 1;

    if(side == MOTOR_LEFT)
    {
        if(speed >= 0)
        {
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_LEFT_FRONT, (uint32_t)pulse_width);
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_LEFT_BACK, 1);
        }
        else
        {
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_LEFT_FRONT, (uint32_t)pulse_width );
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_LEFT_BACK, MOTOR_PWM_PERIOD);
        }
    }
    else
    {
        if(speed >= 0)
        {
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_RIGHT_FRONT, (uint32_t)pulse_width);
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_RIGHT_BACK, 1);
        }
        else
        {
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_RIGHT_FRONT, (uint32_t)pulse_width);
            PWMPulseWidthSet(MOTOR_PWM_BASE, MOTOR_PWM_OUT_RIGHT_BACK, MOTOR_PWM_PERIOD);
        }
    }
}
