/*
 * Quadrature_Encoder.h
 *
 *  Created on: 2017Äê8ÔÂ23ÈÕ
 *      Author: Arthur
 */

#ifndef COMPONENTS_QUADRATURE_ENCODER_H_
#define COMPONENTS_QUADRATURE_ENCODER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "../components/Motor.h"

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


typedef struct
{
    uint32_t periph;
    uint32_t base;
    uint32_t swap_flag;
    uint32_t iinterrupt;

    uint32_t gpio_periph;
    uint32_t gpio_port;
    uint8_t  gpio_pin;
    uint32_t gpio_cfg_a;
    uint32_t gpio_cfg_b;

    int32_t dir;
    int32_t velocity;   /*  unit: pulse edge per second */

}motor_qei_data_t;

extern void Motor_Init_QEI(void);
extern void QEI0_IRQHandler(void);
extern void QEI1_IRQHandler(void);
extern void ConfigureUART0(void);
#endif /* COMPONENTS_QUADRATURE_ENCODER_H_ */
