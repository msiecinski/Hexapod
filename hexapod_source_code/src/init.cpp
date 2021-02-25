#include <Global_defines.hpp>
#include <Init.hpp>
#include <Timefunc.hpp>
#include <MPU6050.h>
#include <TimerThree.h>
#include <TimerOne.h>

extern MPU6050 imu;     //MPU6050 object
IntervalTimer myTimer;  //object for IntervalTimer

void SetupLeg(void)
{
    pinMode(LEG6_1, OUTPUT);
    pinMode(LEG6_2, OUTPUT);
    pinMode(LEG6_3, OUTPUT);

    pinMode(LEG5_1, OUTPUT);
    pinMode(LEG5_2, OUTPUT);
    pinMode(LEG5_3, OUTPUT);

    pinMode(LEG4_1, OUTPUT);
    pinMode(LEG4_2, OUTPUT);
    pinMode(LEG4_3, OUTPUT);

    pinMode(LEG3_1, OUTPUT);
    pinMode(LEG3_2, OUTPUT);
    pinMode(LEG3_3, OUTPUT);

    pinMode(LEG2_1, OUTPUT);
    pinMode(LEG2_2, OUTPUT);
    pinMode(LEG2_3, OUTPUT);

    pinMode(LEG1_1, OUTPUT);
    pinMode(LEG1_2, OUTPUT);
    pinMode(LEG1_3, OUTPUT);
}

void SetupLegSensor(void)
{
    pinMode(SENSOR_1, INPUT_PULLUP);
    pinMode(SENSOR_2, INPUT_PULLUP);
    pinMode(SENSOR_3, INPUT_PULLUP);
    pinMode(SENSOR_4, INPUT_PULLUP);
    pinMode(SENSOR_5, INPUT_PULLUP);
    pinMode(SENSOR_6, INPUT_PULLUP);
}

void SetupLed(void)
{
    pinMode(ADC_LED_1, OUTPUT);
    pinMode(ADC_LED_2, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

void SetupAnalog(void)
{
    pinMode(ADC_SENSOR_1, INPUT);
    pinMode(ADC_SENSOR_2, INPUT);
}

void SetupSharp(void)
{
    pinMode(SHARP_SENSOR, INPUT);
}

void SetupTimer3(int time_us)
{
    Timer3.initialize(time_us);
    Timer3.attachInterrupt(PWM50Hz); 
}

void SetupTimer1(int time_us)
{
    Timer1.initialize(time_us);
    Timer1.attachInterrupt(ReadFunc); 
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
    if(1 == enable)
    {
        interrupts();
    }
    else
    {
       noInterrupts(); 
    }
}