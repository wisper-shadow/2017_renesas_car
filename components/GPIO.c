/*
 * GPIO.c
 *
 *  Created on: 2017��8��24��
 *      Author: Arthur
 */

#include "GPIO.h"

void GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
}
