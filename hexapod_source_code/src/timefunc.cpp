#include <Global_defines.hpp>
#include <Timefunc.hpp>


extern std::array<hexapod, 6> hexapodControl;
extern volatile uint32_t delayFlag;
extern volatile int adc[2];
extern int sharpSensor;

volatile uint32_t dutyBuff[6][3];

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
        if (counter ==  dutyBuff[5][0]) 
        {
            digitalWrite(LEG6_1, LOW);
        }
        if (counter == dutyBuff[5][1]) 
        {
            digitalWrite(LEG6_2, LOW);
        }
        if (counter == dutyBuff[5][2]) 
        {
            digitalWrite(LEG6_3, LOW);
        }
        if (counter ==  dutyBuff[4][0]) 
        {
            digitalWrite(LEG5_1, LOW);
        }
        if (counter ==  dutyBuff[4][1]) 
        {
            digitalWrite(LEG5_2, LOW);
        }
        if (counter ==  dutyBuff[4][2]) 
        {
            digitalWrite(LEG5_3, LOW);
        }
        if (counter ==  dutyBuff[3][0]) 
        {
            digitalWrite(LEG4_1, LOW);
        }
        if (counter ==  dutyBuff[3][1]) 
        {
            digitalWrite(LEG4_2, LOW);
        }
        if (counter ==  dutyBuff[3][2]) 
        {
            digitalWrite(LEG4_3, LOW);
        }
        if (counter ==  dutyBuff[2][0]) 
        {
            digitalWrite(LEG3_1, LOW);
        }
        if (counter ==  dutyBuff[2][1]) 
        {
            digitalWrite(LEG3_2, LOW);
        }
        if (counter ==  dutyBuff[2][2]) 
        {
            digitalWrite(LEG3_3, LOW);
        }
        if (counter ==  dutyBuff[1][0]) 
        {
            digitalWrite(LEG2_1, LOW);
        }
        if (counter ==  dutyBuff[1][1]) 
        {
            digitalWrite(LEG2_2, LOW);
        }
        if (counter ==  dutyBuff[1][2]) 
        {
            digitalWrite(LEG2_3, LOW);
        }
        if (counter ==  dutyBuff[0][0]) 
        {
            digitalWrite(LEG1_1, LOW);
        }
        if (counter ==  dutyBuff[0][1]) 
        {
            digitalWrite(LEG1_2, LOW);
        }
        if (counter ==  dutyBuff[0][2]) 
        {
            digitalWrite(LEG1_3, LOW);
        }
    }
}

void DelayFunc(void)
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

void ReadFunc(void)
{
    static int counter = 0;
    static int ledState = LOW;
    if(counter>=9)
    {   
        adc[0] = analogRead(ADC_SENSOR_1);
        adc[1] = analogRead(ADC_SENSOR_2);
        
        if (ledState == LOW) {
            ledState = HIGH;
        } else {
            ledState = LOW;
        }
        digitalWrite(LED_BUILTIN, ledState);
        counter = 0;
    }
    else
    {
        counter++;
    }
    sharpSensor = digitalRead(SHARP_SENSOR);
}