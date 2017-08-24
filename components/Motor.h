/*
 * Motor.h
 *
 *  Created on: 2017Äê8ÔÂ22ÈÕ
 *      Author: Arthur
 */

#ifndef COMPONENTS_MOTOR_H_
#define COMPONENTS_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT =1,
}motor_side_e;

extern void Motor_PWM_Init(void);
extern void Motor_Set_Throttle(motor_side_e side, int pulse_width);     //pluse_width range: -400 - 400

#endif /* COMPONENTS_MOTOR_H_ */
