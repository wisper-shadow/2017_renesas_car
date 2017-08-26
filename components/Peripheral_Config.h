/*
 * Peripheral_Config.h
 *
 *  Created on: 2017Äê8ÔÂ26ÈÕ
 *      Author: Arthur
 */

#ifndef COMPONENTS_PERIPHERAL_CONFIG_H_
#define COMPONENTS_PERIPHERAL_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
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

#include "PID.h"

extern bool is_alarm;

extern void Peripheral_Enable(void);
extern void Priority_Set(void);
extern void UART0_IntHandler(void);
extern void UART1_IntHandler(void);
extern void UART2_IntHandler(void);

#endif /* COMPONENTS_PERIPHERAL_CONFIG_H_ */
