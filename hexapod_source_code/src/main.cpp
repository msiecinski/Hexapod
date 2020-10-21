
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
    startup.xyz = {-0,10.5,-4};
    startup.delay = 1;

    Serial.begin(115200);         //olny in debug version
    SetupTimer3(PWMTIMEBASE);     //interupt every (TimeBase) us
    DelayTimer(DELAYTIMEBASE);
    SetupLeg();
    gyroStatus = SetupMPU6050();
    SetupPS4();
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