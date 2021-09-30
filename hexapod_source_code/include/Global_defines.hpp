#ifndef Global_defines_H_
#define Global_defines_H_

#include <cmath>
#include <array>
#include <Arduino.h>
#include <Leg.hpp>

/*nesessery structs */

    struct angle
    {
        volatile double q1;
        volatile double q2;
        volatile double q3;
    };
    struct position
    {
        volatile double x;
        volatile double y;
        volatile double z;
    };
    struct  hexapod
    {
        angle angles;                                   //calculated angles
        position xyz;                                   //set postion
        position offset;                                //offset postion leg(when "no" ground)
        volatile uint32_t duty[3];                      //duty for pwm
        volatile uint32_t delay;                        //delay value flag
        volatile uint32_t update;                       //update value flag
    };

    enum movetype {forvard = 0, backward, forvardleft, forvardright, left, right, up, down,wider,narrower };

    //constants based on the robot dimensions 
    #define A1   (5)
    #define A2   (15)
    #define A3   (25)//(22)
    #define E    (2)

    #define CIRCLEVAL           2                       //diff ofset for turning
    #define BASEHEIGHT          (-5)
    #define MOVEHEIGHT          8
    #define STEPSIZE            5
    #define BASEWIDTH           10

    #define CALIBRATION         7                       //calibration interupt for 50Hz pwm
    #define INTERRUPTCOUNTER    (4000 - CALIBRATION)    //interupt counter for 50Hz pwm
    #define LIMITHI             (500 + 11)              //limit for skip conditions after every 2.5ms //11 bcs of 0x1FF
    
    #define DELAYTIMER1         50000                   //!!!!!!!!!!!!!!!!!!!!!!!
    #define PWMTIMEBASE         5                       //time base for interrupts
    #define DELAYTIMEBASE       1000                    //time base for delay interrupts
    #define DELAYCOUNTERMAXVAL  10                     //max value for delay_counter
       
    #define VERTICALMOVEDELAY   2                       //delays for vertical legs move
    #define HORIZONTALMOVEDELAY 5                      //delays for horizontal legs move
     //constants for bool fucntions
    #define TRUE                1          
    #define FALSE               0
    
    #define HWSERIAL            Serial7                 //Serial port for bluetooth 

#endif /* Global_defines_H_ */

/*
    Names convention
    -defines with a capital letter,
    -variable names - first "member" lowercase,
    -functions names - every "name member" with a capital letter,
    -files .hpp with a capital letter,
    -files .cpp with a lowercase letter,
    -spaces for value assignments and comparisons
*/