/*
 * PID.c
 *
 *  Created on: 2017Äê8ÔÂ24ÈÕ
 *      Author: Arthur
 */

#include "PID.h"

PID_Params PID_Left_Motor;
PID_Cfg PID_Left_Motor_Cfg;

PID_Params PID_Right_Motor;
PID_Cfg PID_Right_Motor_Cfg;

Dest_Vel Dest_Motor_Vel;

uint32_t time_stamp = 0;

bool Start_Left_PID = false;
bool Start_Right_PID = false;

void PID_Init(void)
{
    Dest_Motor_Vel.dest_left            = Dest_Left_Vel;
    Dest_Motor_Vel.dest_right           = Dest_Right_Vel;
    PID_Left_Motor.Kp                   = DEFAULT_KP;
    PID_Left_Motor.Ki                   = DEFAULT_KI;
    PID_Left_Motor.Kd                   = DEFAULT_KD;
    PID_Left_Motor.dt                   = (float)(1.0/DEFAULT_PID_FREQ);
    PID_Left_Motor.I_MAX                = DEFAULT_I_MAX;
    PID_Left_Motor.d_LPF                = DEFAULT_LPFITER;
    PID_Left_Motor.OUT_MAX              = DEFAULT_OUT_MAX;

    PID_Left_Motor_Cfg.Error            = 0;
    PID_Left_Motor_Cfg.LastError        = 0;
    PID_Left_Motor_Cfg.Proportion       = 0;
    PID_Left_Motor_Cfg.Integrator       = 0;
    PID_Left_Motor_Cfg.Derivative       = 0;
    PID_Left_Motor_Cfg.Last_Derivative  = 0;
    PID_Left_Motor_Cfg.PID_OUT          = 0;
}

void Motor_PID(void)
{
    PID_Left_Motor_Cfg.Proportion = PID_Left_Motor.Kp * PID_Left_Motor_Cfg.Error;

    // calculate integrator constrain in i_max.
    if((PID_Left_Motor.Ki != 0) && (PID_Left_Motor.dt != 0))
    {
        PID_Left_Motor_Cfg.Integrator += PID_Left_Motor.Ki * PID_Left_Motor_Cfg.Error * PID_Left_Motor.dt;
        if (PID_Left_Motor_Cfg.Integrator < -(PID_Left_Motor.I_MAX))
        {
            PID_Left_Motor_Cfg.Integrator = -(PID_Left_Motor.I_MAX);
        }
        else if (PID_Left_Motor_Cfg.Integrator > PID_Left_Motor.I_MAX)
        {
            PID_Left_Motor_Cfg.Integrator = PID_Left_Motor.I_MAX;
        }
    }
    else
    {
        PID_Left_Motor_Cfg.Integrator = 0;
    }

    // calculate instantaneous Derivative.
    if((PID_Left_Motor.Kd != 0) && (PID_Left_Motor.dt != 0))
    {
        PID_Left_Motor_Cfg.Derivative = PID_Left_Motor.Kd * (PID_Left_Motor_Cfg.Error - PID_Left_Motor_Cfg.LastError) / PID_Left_Motor.dt;
        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy.
        PID_Left_Motor_Cfg.Derivative = PID_Left_Motor_Cfg.Last_Derivative + (PID_Left_Motor.dt / ( PID_Left_Motor.d_LPF + PID_Left_Motor.dt)) * (PID_Left_Motor_Cfg.Derivative - PID_Left_Motor_Cfg.Last_Derivative);
        // update state
        PID_Left_Motor_Cfg.LastError = PID_Left_Motor_Cfg.Error;
        PID_Left_Motor_Cfg.Last_Derivative = PID_Left_Motor_Cfg.Derivative;
    }

    PID_Left_Motor_Cfg.PID_OUT = PID_Left_Motor_Cfg.Proportion + PID_Left_Motor_Cfg.Integrator + PID_Left_Motor_Cfg.Derivative;

    if (PID_Left_Motor_Cfg.PID_OUT > PID_Left_Motor.OUT_MAX)
    {
        PID_Left_Motor_Cfg.PID_OUT = PID_Left_Motor.OUT_MAX;
    }
    else if (PID_Left_Motor_Cfg.PID_OUT < -(PID_Left_Motor.OUT_MAX))
    {
        PID_Left_Motor_Cfg.PID_OUT = -(PID_Left_Motor.OUT_MAX);
    }
}

void Set_Dest_Vel(int left, int right)
{
    Dest_Motor_Vel.dest_left = left;
    Dest_Motor_Vel.dest_right = right;
}

void Timer0A_IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    time_stamp++;
    if(time_stamp == 5)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    if(time_stamp == 10)
    {
        time_stamp = 0;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    }
    PID_Left_Motor_Cfg.Error = Dest_Motor_Vel.dest_left - qei_data_array[0].velocity / 104;
    PID_Right_Motor_Cfg.Error = Dest_Motor_Vel.dest_right - qei_data_array[1].velocity / 104;
    Motor_PID();
//    if(Start_Left_PID)
        Motor_Set_Throttle(MOTOR_LEFT, (int)PID_Left_Motor_Cfg.PID_OUT);
//    if(Start_Right_PID)
//        Motor_Set_Throttle(MOTOR_RIGHT, PID_Right_Motor_Cfg.PID_OUT);
}
