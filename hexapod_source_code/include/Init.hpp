#ifndef Init_H_
#define Init_H_ 
 
 
 //Functions for setup MCU
   void SetupTimer3(int);                   //using for generate pwm signal for servos
   void SetupUart(int);                    //UART for wifi or bluetooth  module
   _Bool SetupMPU6050(void);                //I2C setup and connect to MPU6050 (Pins 18 and 19(A4 and A5))
   void InterruptEnable(bool);             //for on/off interrupts
   void DelayTimer(int);                   //using for system "clock"
   void SetupLeg(void);                    //settup "leg" pins as outputs
   void SetupLegSensor(void);              //settup "leg sensor" pins as pullup inputs
    

    #endif /* Init_H_ */