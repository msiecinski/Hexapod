#include <Global_defines.hpp>
#include <Init.hpp>
#include <Timefunc.hpp>
#include <MPU6050.h>
#include <TimerThree.h>

extern MPU6050 imu;     //MPU6050 object
IntervalTimer myTimer;  //object for IntervalTimer

void SetupLeg(void)
{
    pinMode(LEG6_1, OUTPUT);
    pinMode(LEG6_2, OUTPUT);
    pinMode(LEG5_1, OUTPUT);
    pinMode(LEG4_3, OUTPUT);
    pinMode(LEG4_2, OUTPUT);
    pinMode(LEG4_1, OUTPUT);
    pinMode(LEG3_3, OUTPUT);
    pinMode(LEG3_2, OUTPUT);
}

void SetupTimer3(int time_us)
{
    Timer3.initialize(time_us);
    Timer3.attachInterrupt(PWM50Hz); 
}

void SetupUart(int baud)
{
    Serial1.begin(baud);
}

_Bool SetupMPU6050(void)
{  
    Wire.begin();
    
    if (!imu.begin(AFS_2G, GFS_250DPS)) 
    {
#ifdef _DEBUG_
        Serial.println("MPU6050 is online...");
#endif
        return TRUE;
    }
    else 
    {
#ifdef _DEBUG_
        Serial.println("Failed to init MPU6050");
#endif
        return FALSE;
    }
}

void DelayTimer(int time_us)
{
    pinMode(LED_BUILTIN, OUTPUT);
    myTimer.begin(DelayFunc, time_us);
}

void InterruptEnable(bool enable)
{ 
    if(1==enable)
    {
        interrupts();
    }
    else
    {
       noInterrupts(); 
    }
}