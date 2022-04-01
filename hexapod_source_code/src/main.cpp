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
   // if(i == 3 || i == 0)
      tmp.xyz = {0,15,BASEHEIGHT};
    //else
     //
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     RotateCordinate(i,tmp.xyz,0,15);
    SetOneLeg(i,tmp);
    tmp = startup;
  }  
}
void loop2() {
  int incomingByte;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("USB received: ");
    Serial.println(incomingByte, DEC);
    HWSERIAL.print("USB received:");
    HWSERIAL.println(incomingByte, DEC);
  }
  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
    HWSERIAL.print("UART received:");
    HWSERIAL.println(incomingByte, DEC);
  }
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
  int static status = 0;
  int incomingByte;
  if (HWSERIAL.available() > 0) {
      incomingByte = HWSERIAL.read();
  }
  if(incomingByte == 97)
    status = 5;
  if(incomingByte == 102)
    status = 0;
  if(status == 5)
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