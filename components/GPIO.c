/*
 * GPIO.c
 *
 *  Created on: 2017Äê8ÔÂ24ÈÕ
 *      Author: Arthur
 */

#include "GPIO.h"

void GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}
