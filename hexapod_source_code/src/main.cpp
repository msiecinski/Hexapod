
#include <Global_defines.hpp>
#include <Init.hpp>
#include <Kinematics.hpp>
#include <Movement.hpp>

std::array<hexapod, 6> hexapodControl;
_Bool gyroStatus = 0;

void setup(void)
{
    hexapod startup;
    startup.xyz = {0,0,0};
    startup.delay = 1;

    Serial.begin(115200);         //olny in debug version
    SetupTimer3(PWMTIMEBASE);     //interupt every (TimeBase) us
    DelayTimer(DELAYTIMEBASE);
    SetupLeg();
    gyroStatus = SetupMPU6050();
   
    for(int i=0;i<6;i++)
     {
       while(!SetPosition(i,startup));
     }  
}

void loop(void)
{
  while(1)
  {
   Move(forvard,3);
  }
}