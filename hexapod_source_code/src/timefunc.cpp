#include <Global_defines.hpp>
#include <Timefunc.hpp>


extern std::array<hexapod, 6> hexapodControl;
extern volatile uint32_t delayFlag;

void PWM50Hz(void)
{   
    static volatile uint32_t counter = 0;

    if(counter < (INTERRUPTCOUNTER))
		counter++;
	else
    {
		counter = 0;
        digitalWrite(LEG6_1, HIGH);
        digitalWrite(LEG6_2, HIGH);
        digitalWrite(LEG6_3, HIGH);

        digitalWrite(LEG5_1, HIGH);
        digitalWrite(LEG5_2, HIGH);
        digitalWrite(LEG5_3, HIGH);

        digitalWrite(LEG4_1, HIGH);
        digitalWrite(LEG4_2, HIGH);
        digitalWrite(LEG4_3, HIGH);

        digitalWrite(LEG3_1, HIGH);
        digitalWrite(LEG3_2, HIGH);
        digitalWrite(LEG3_3, HIGH);

        digitalWrite(LEG2_1, HIGH);
        digitalWrite(LEG2_2, HIGH);
        digitalWrite(LEG2_3, HIGH);

        digitalWrite(LEG1_1, HIGH);
        digitalWrite(LEG1_2, HIGH);
        digitalWrite(LEG1_3, HIGH);
    }

    if(counter < (LIMITHI+1))   //skip all by~87.5% time(17.5ms)
    {   
        if (counter ==  hexapodControl[5].duty[0]) 
        {
            digitalWrite(LEG6_1, LOW);
        }
        if (counter == hexapodControl[5].duty[1]) 
        {
            digitalWrite(LEG6_2, LOW);
        }
        if (counter == hexapodControl[5].duty[2]) 
        {
            digitalWrite(LEG6_3, LOW);
        }
        if (counter ==  hexapodControl[4].duty[0]) 
        {
            digitalWrite(LEG5_1, LOW);
        }
        if (counter ==  hexapodControl[4].duty[1]) 
        {
            digitalWrite(LEG5_2, LOW);
        }
        if (counter ==  hexapodControl[4].duty[2]) 
        {
            digitalWrite(LEG5_3, LOW);
        }
        if (counter ==  hexapodControl[3].duty[0]) 
        {
            digitalWrite(LEG4_1, LOW);
        }
        if (counter ==  hexapodControl[3].duty[1]) 
        {
            digitalWrite(LEG4_2, LOW);
        }
        if (counter ==  hexapodControl[3].duty[2]) 
        {
            digitalWrite(LEG4_3, LOW);
        }
        if (counter ==  hexapodControl[2].duty[0]) 
        {
            digitalWrite(LEG3_1, LOW);
        }
        if (counter ==  hexapodControl[2].duty[1]) 
        {
            digitalWrite(LEG3_2, LOW);
        }
        if (counter ==  hexapodControl[2].duty[2]) 
        {
            digitalWrite(LEG3_3, LOW);
        }
        if (counter ==  hexapodControl[1].duty[0]) 
        {
            digitalWrite(LEG2_1, LOW);
        }
        if (counter ==  hexapodControl[1].duty[1]) 
        {
            digitalWrite(LEG2_2, LOW);
        }
        if (counter ==  hexapodControl[1].duty[2]) 
        {
            digitalWrite(LEG2_3, LOW);
        }
        if (counter ==  hexapodControl[0].duty[0]) 
        {
            digitalWrite(LEG1_1, LOW);
        }
        if (counter ==  hexapodControl[0].duty[1]) 
        {
            digitalWrite(LEG1_2, LOW);
        }
        if (counter ==  hexapodControl[0].duty[2]) 
        {
            digitalWrite(LEG1_3, LOW);
        }
    }
}

void DelayFunc()
{
   /*   
        Implementation delay function
        (used by function SetPosition())
        elay = delayFlag(variable) * DELAYTIMEBASE(const) * DELAYCOUNTERMAXVAL(const) (us)
    */
    static uint32_t delayCounter = 0;

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