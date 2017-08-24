/*
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

#include "components/Timer.h"
#include "components/Motor.h"
#include "components/Quadrature_Encoder.h"

void main(void)
 {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    ConfigureUART0();
    Motor_PWM_Init();
    Motor_Init_QEI();
    Motor_Set_Throttle(MOTOR_LEFT, 0);
    while(1)
    {
        ;
    }
}
