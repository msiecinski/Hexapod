#include <Global_defines.hpp>
#include <Init.hpp>
#include <Kinematics.hpp>
#include <Movement.hpp>
#include <Gyroscope.hpp>

std::array<hexapod, 6> hexapodControl;
_Bool gyroStatus = 0;
volatile int stepSize = STEPSIZE;
volatile int baseHeight = BASEHEIGHT;
volatile int baseWidth = BASEWIDTH;
volatile int adc[2]= {0,0};
volatile int sharpSensor = 0;

void setup(void)
{
  hexapod startup, tmp;
  //startup.xyz = {-0,baseHeight,baseWidth};
  //startup.xyz = {-0,19.5,-4};
  //startup.xyz = {0,0,-8};       //starting leg position
  startup.xyz = {0,0,BASEHEIGHT};       //starting leg position
  //startup.offset = {0,0,0};
  startup.delay = 1;            //added little delay after all legs @startup
  startup.offset = {0,0,0};     //clear all offsets
  startup.update = 0;           //flag for "push" to buffer
  tmp = startup;
  SetupLeg();                   //setup pins using to generate pwm signal for each leg joint
  SetupLegSensor();             //setup pins using to ground check
  SetupLed();                   //setup "led" pins
  SetupAnalog();                //setup ADC 
  SetupSharp();                 //setup pin connect to sharp
  SetupUart(9600);              //UART for bluetooth
  SetupTimer1(DELAYTIMER1);     //interupt every (DELAYTIMER1) us
  SetupTimer3(PWMTIMEBASE);     //interupt every (TimeBase) us
  DelayTimer(DELAYTIMEBASE);    //interupt using for generate delays
  gyroStatus = SetupMPU6050();  //setup gyroscope(using in future)
  Serial.begin(115200);         //olny in debug version
  for(int i=0;i<6;i++)
  {
    if(i == 3 || i == 0)
      tmp.xyz = {0,19,BASEHEIGHT};
    else
      RotateCordinate(i,tmp.xyz,0,19);
    SetLeg(i,tmp);
    tmp = startup;
  }  
  while(!PushBuff());
}

void loop(void)
{
 // while(1){};
  // SlideHorizontal(-30,0);
  //
  static movetype lastDir,actDir;
  actDir = forvard;
  lastDir = forvard;
  //Move(backward,5);
  //delay(500);
  //PrepareWalk(forvard);
  while(1)
  {

    if(1)
    {
      actDir = forvard;
      if(lastDir!=actDir)
      {
        PrepareWalk(actDir);
        lastDir = actDir;
      }
      Move(actDir,5);
    }
    if(0)
    {
      actDir = backward;
      if(lastDir!=actDir)
      {
        PrepareWalk(actDir);
        lastDir = actDir;
      }
      Move(actDir,5);
    }

    //if(sharpSensor==1)
    if(adc[1]  < 910)
      digitalWrite(ADC_LED_2, HIGH);
    else
      digitalWrite(ADC_LED_2, LOW);
    if(adc[0]  < 910)
      digitalWrite(ADC_LED_1, HIGH);
    else
      digitalWrite(ADC_LED_1, LOW);
    //GyroReadAngle();
    //MoveAtPlace(down,1);
    //elay(500);
  }
}