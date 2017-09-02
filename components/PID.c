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

uint32_t time_stamp_0A = 0;

bool Start_Left_PID = true;
bool Start_Right_PID = true;

uint8_t throttle_Left_90 = 40;
uint8_t throttle_Right_90 = 40;

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

    PID_Right_Motor.Kp                   = DEFAULT_KP;
    PID_Right_Motor.Ki                   = DEFAULT_KI;
    PID_Right_Motor.Kd                   = DEFAULT_KD;
    PID_Right_Motor.dt                   = (float)(1.0/DEFAULT_PID_FREQ);
    PID_Right_Motor.I_MAX                = DEFAULT_I_MAX;
    PID_Right_Motor.d_LPF                = DEFAULT_LPFITER;
    PID_Right_Motor.OUT_MAX              = DEFAULT_OUT_MAX;

    PID_Right_Motor_Cfg.Error            = 0;
    PID_Right_Motor_Cfg.LastError        = 0;
    PID_Right_Motor_Cfg.Proportion       = 0;
    PID_Right_Motor_Cfg.Integrator       = 0;
    PID_Right_Motor_Cfg.Derivative       = 0;
    PID_Right_Motor_Cfg.Last_Derivative  = 0;
    PID_Right_Motor_Cfg.PID_OUT          = 0;
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

    PID_Right_Motor_Cfg.Proportion = PID_Right_Motor.Kp * PID_Right_Motor_Cfg.Error;

    // calculate integrator constrain in i_max.
    if((PID_Right_Motor.Ki != 0) && (PID_Right_Motor.dt != 0))
    {
        PID_Right_Motor_Cfg.Integrator += PID_Right_Motor.Ki * PID_Right_Motor_Cfg.Error * PID_Right_Motor.dt;
        if (PID_Right_Motor_Cfg.Integrator < -(PID_Right_Motor.I_MAX))
        {
            PID_Right_Motor_Cfg.Integrator = -(PID_Right_Motor.I_MAX);
        }
        else if (PID_Right_Motor_Cfg.Integrator > PID_Right_Motor.I_MAX)
        {
            PID_Right_Motor_Cfg.Integrator = PID_Right_Motor.I_MAX;
        }
    }
    else
    {
        PID_Right_Motor_Cfg.Integrator = 0;
    }

    // calculate instantaneous Derivative.
    if((PID_Right_Motor.Kd != 0) && (PID_Right_Motor.dt != 0))
    {
        PID_Right_Motor_Cfg.Derivative = PID_Right_Motor.Kd * (PID_Right_Motor_Cfg.Error - PID_Right_Motor_Cfg.LastError) / PID_Right_Motor.dt;
        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy.
        PID_Right_Motor_Cfg.Derivative = PID_Right_Motor_Cfg.Last_Derivative + (PID_Right_Motor.dt / ( PID_Right_Motor.d_LPF + PID_Right_Motor.dt)) * (PID_Right_Motor_Cfg.Derivative - PID_Right_Motor_Cfg.Last_Derivative);
        // update state
        PID_Right_Motor_Cfg.LastError = PID_Right_Motor_Cfg.Error;
        PID_Right_Motor_Cfg.Last_Derivative = PID_Right_Motor_Cfg.Derivative;
    }

    PID_Right_Motor_Cfg.PID_OUT = PID_Right_Motor_Cfg.Proportion + PID_Right_Motor_Cfg.Integrator + PID_Right_Motor_Cfg.Derivative;

    if (PID_Right_Motor_Cfg.PID_OUT > PID_Right_Motor.OUT_MAX)
    {
        PID_Right_Motor_Cfg.PID_OUT = PID_Right_Motor.OUT_MAX;
    }
    else if (PID_Right_Motor_Cfg.PID_OUT < -(PID_Right_Motor.OUT_MAX))
    {
        PID_Right_Motor_Cfg.PID_OUT = -(PID_Right_Motor.OUT_MAX);
    }
}

void PID_Clear(void)
{
    PID_Left_Motor_Cfg.Integrator = 0;
    PID_Right_Motor_Cfg.Integrator = 0;
}

void Timer0A_IntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = TimerIntStatus(TIMER0_BASE, true);
    TimerIntClear(TIMER0_BASE, ui32Status);

    time_stamp_0A++;
    if(time_stamp_0A == 5)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        if(is_alarm)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
        }
    }

    if(time_stamp_0A == 10)
    {
        time_stamp_0A = 0;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    }

    PID_Left_Motor_Cfg.Error = Dest_Motor_Vel.dest_left - qei_data_array[0].velocity / 104;
    PID_Right_Motor_Cfg.Error = Dest_Motor_Vel.dest_right - qei_data_array[1].velocity / 104;
    Motor_PID();
    if(Start_Left_PID)
        Motor_Set_Throttle(MOTOR_LEFT, (int)PID_Left_Motor_Cfg.PID_OUT);
    if(Start_Right_PID)
        Motor_Set_Throttle(MOTOR_RIGHT, (int)PID_Right_Motor_Cfg.PID_OUT);
}

int Max(int a, int b)
{
    if(a >= b)
        return a;
    if(a < b)
        return b;
}

int Min(int a, int b)
{
    if(a >= b)
        return b;
    if(a < b)
        return a;
}

void Key_GO(void)
{
//    PID_Clear();
    if(Dest_Motor_Vel.dest_left == Dest_Motor_Vel.dest_right)
    {
        Dest_Motor_Vel.dest_left += 10;
        Dest_Motor_Vel.dest_right += 10;
        if(Dest_Motor_Vel.dest_left > 100)
            Dest_Motor_Vel.dest_left = 100;
        if(Dest_Motor_Vel.dest_right > 100)
            Dest_Motor_Vel.dest_right = 100;
    }
    else
    {
        Dest_Motor_Vel.dest_left = Max(Dest_Motor_Vel.dest_left, Dest_Motor_Vel.dest_right);
        Dest_Motor_Vel.dest_right = Dest_Motor_Vel.dest_left;
    }
}

void Key_BACK(void)
{
//    PID_Clear();
    if(Dest_Motor_Vel.dest_left == Dest_Motor_Vel.dest_right)
    {
        Dest_Motor_Vel.dest_left -= 10;
        Dest_Motor_Vel.dest_right -= 10;
        if(Dest_Motor_Vel.dest_left < -100)
            Dest_Motor_Vel.dest_left = -100;
        if(Dest_Motor_Vel.dest_right < -100)
            Dest_Motor_Vel.dest_right = -100;
    }
    else
    {
        Dest_Motor_Vel.dest_left = Min(Dest_Motor_Vel.dest_left, Dest_Motor_Vel.dest_right);
        Dest_Motor_Vel.dest_right = Dest_Motor_Vel.dest_left;
    }
}

void Key_LEFT(void)
{
//    PID_Clear();
    Dest_Motor_Vel.dest_left -= 2;
    if(Dest_Motor_Vel.dest_left < -100)
        Dest_Motor_Vel.dest_left = -100;
}

void Key_RIGHT(void)
{
//    PID_Clear();
    Dest_Motor_Vel.dest_right -= 2;
    if(Dest_Motor_Vel.dest_right < -100)
        Dest_Motor_Vel.dest_right = -100;
}

void Key_STOP(void)
{
    PID_Clear();
    Start_Left_PID = true;
    Start_Right_PID = true;
    throttle_Left_90 = 50;
    throttle_Right_90 = 50;
    Dest_Motor_Vel.dest_left = 0;
    Dest_Motor_Vel.dest_right = 0;
}

void Key_LEFT_90(void)
{
    PID_Clear();
//    Dest_Motor_Vel.dest_left = -5;
//    Dest_Motor_Vel.dest_right = 5;
    Start_Left_PID = false;
    Start_Right_PID = false;
    Motor_Set_Throttle(MOTOR_LEFT, -throttle_Left_90);
    Motor_Set_Throttle(MOTOR_RIGHT, throttle_Left_90);
    throttle_Left_90++;

}

void Key_RIGHT_90(void)
{
    PID_Clear();
//    Dest_Motor_Vel.dest_left = 5;
//    Dest_Motor_Vel.dest_right = -5;
    Start_Left_PID = false;
    Start_Right_PID = false;
    Motor_Set_Throttle(MOTOR_LEFT, throttle_Right_90);
    Motor_Set_Throttle(MOTOR_RIGHT, -throttle_Right_90);
    throttle_Right_90++;
}
