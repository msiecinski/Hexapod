#include <Global_defines.hpp>
#include <Timefunc.hpp>


extern std::array<hexapod, 6> hexapodControl;
extern double angle_x,angle_y;
extern volatile uint32_t delayFlag;

void PWM50Hz(void)
{   
    static volatile int counter = 0;
    static volatile int tmp = 100;
    static volatile int tmp_angle = 100;
    static volatile int tmpy = 100;
    static volatile int tmp_angley = 100;

	if(counter < (INTERRUPTCOUNTER))
		counter++;
	else{
		counter = 0;
        digitalWrite(LEG6_1, HIGH);
        digitalWrite(LEG6_2, HIGH);
        digitalWrite(LEG5_1, HIGH);
        digitalWrite(LEG4_3, HIGH);
        digitalWrite(LEG4_2, HIGH);
        digitalWrite(LEG4_1, HIGH);
        digitalWrite(LEG3_3, HIGH);
        digitalWrite(LEG3_2, HIGH);
	}
    tmp_angle=angle_x;
    if(tmp_angle<0)
        tmp_angle*=-1;
    tmp=(int)(100+tmp_angle*4.4);
    if(tmp<100)
        tmp=100;
    if(tmp>500)
        tmp=500;
     tmp_angley=angle_y;
    if(tmp_angley<0)
        tmp_angley*=-1;
    tmpy=(int)(100+tmp_angley*4.4);
    if(tmpy<100)
        tmpy=100;
    if(tmpy>500)
        tmpy=500;
    if(counter < (LIMITHI+1))   //skip all by~87.5% time(17.5ms)
    {   
        if (counter ==  tmp) 
        {
            digitalWrite(LEG6_1, LOW);
        }
        if (counter  ==  tmpy)
        {
            digitalWrite(LEG6_2, LOW);
        }
        if (counter ==  500) 
        {
            digitalWrite(LEG5_1, LOW);
        }
        if (counter ==  350) 
        {
            digitalWrite(LEG4_3, LOW);
        }
        if (counter ==  222) 
        {
            digitalWrite(LEG4_2, LOW);
        }
        if (counter ==  hexapodControl[1].duty[0]) 
        {
            digitalWrite(LEG4_1, LOW);
        }
        if (counter ==  hexapodControl[1].duty[1]) 
        {
            digitalWrite(LEG3_3, LOW);
        }
        if (counter ==  hexapodControl[1].duty[2]) 
        {
            digitalWrite(LEG3_2, LOW);
        }
    }
}


int ledState = LOW;
volatile unsigned long blinkCount = 0; // use volatile for shared variables


void DelayFunc()
 {
    static uint32_t delayCounter = 0;
    if (ledState == LOW) {
        ledState = HIGH;
        blinkCount = blinkCount + 1;  // increase when LED turns on
    } 
    else
    {
        ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
    /*
    Implementation delay function
    (used by function SetPosition())
    delay = DelayBase * DelayCounterMaxVal (us)
    */
    if(delayFlag != 0);
    { 
        if(delayCounter == (delayFlag * DELAYCOUNTERMAXVAL))
        {
            delayFlag = 0;
            delayCounter = 0;
        }
        else
        {
        delayCounter++;
        }
    }
}