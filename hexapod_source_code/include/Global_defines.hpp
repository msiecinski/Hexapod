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
    struct position                         //before named as coordinates
    {
        volatile double x;
        volatile double y;
        volatile double z;
    };
    struct  hexapod
    {
        angle angles;
        position xyz;
        volatile uint32_t duty[3];                           //duty for pwm
        volatile uint32_t delay;                        //delay value flag
    };

    enum movetype {forvard = 0, backward, forvardleft, forvardright, left, right};

    //constants based on the robot dimensions 
    #define a1   (5)
    #define a2   (7.5)
    #define a3   (2)
    #define E    (2)

    #define CIRCLEVAL           2                       //diff ofset for turning
    #define BASEHEIGHT          (-5)
    #define MOVEHEIGHT          3

    #define CALIBRATION         7                       //calibration interupt for 50Hz pwm
    #define INTERRUPTCOUNTER    (4000 - CALIBRATION)    //interupt counter for 50Hz pwm
    #define LIMITHI             500                     //limit for skip conditions after every 2.5ms
    
    #define PWMTIMEBASE         5                       //time base for interrupts
    #define DELAYTIMEBASE       1000                  //time base for delay interrupts
    #define DELAYCOUNTERMAXVAL  100                       //max value for delay_counter
       
     //constants for bool fucntions
    #define TRUE                1          
    #define FALSE               0


#endif /* Global_defines_H_ */

/*
    Konwencje ogólne
    -define z dużych liter,
    -nazwy zmiennych - pierwszy człon mała litera,
    -nazwy funkcji - każdy kolejny człon z dużej litery,
    -pliki .hpp z dużej,
    -pliki .cpp z małej,
    -spacje przy przypisaniach wartości i porównaniach
*/