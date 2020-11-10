
#include <Global_defines.hpp>
#include <Init.hpp>
#include <Kinematics.hpp>
#include <Movement.hpp>
#include <Ps4control.hpp>

std::array<hexapod, 6> hexapodControl;
_Bool gyroStatus = 0;
volatile int stepSize = STEPSIZE;
volatile int baseHeight = BASEHEIGHT;
volatile int baseWidth = BASEWIDTH;



void setup(void)
{
    hexapod startup;
    //startup.xyz = {-0,baseHeight,baseWidth};
   // startup.xyz = {-0,19.5,-4};
    startup.xyz = {0,35,3};       //starting leg position
    //startup.offset = {0,0,0};
    startup.delay = 1;            //added little delay after all legs @startup
    startup.offset = {0,0,0};     //clear all offsets

    Serial.begin(115200);         //olny in debug version
    SetupTimer3(PWMTIMEBASE);     //interupt every (TimeBase) us
    DelayTimer(DELAYTIMEBASE);    //interupt using for generate delays
    SetupLeg();                   //setup pins using to generate pwm signal for each leg joint
    gyroStatus = SetupMPU6050();  //setup gyroscope(using in future)
    //SetupPS4();
    for(int i=0;i<6;i++)
    {
      while(!SetPosition(i,startup));
    }  
}

void loop(void)
{
  while(1)
  {
    ; 
  }
}