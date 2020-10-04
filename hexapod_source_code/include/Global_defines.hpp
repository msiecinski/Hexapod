#ifndef Global_defines_H_
#define Global_defines_H_

#include <cmath>
#include <array>
#include <Arduino.h>
#include <Leg.hpp>

/*nesessery structs */

    struct angle
    {
        double q1;
        double q2;
        double q3;
    };
    struct position                         //before named as coordinates
    {
        int x;
        int y;
        int z;
    };
    struct  hexapod
    {
        angle angles;
        position xyz;
        int duty[3];                       //duty for pwm
        bool delay;                        //delay value flag
    };

    enum movetype {forvard = 0, backward, forvardleft, forvardright, left, right};

    //constants based on the robot dimensions 
    #define a1   (3)
    #define a2   (5.5)
    #define a3   (8)
    #define E    (2)

    #define CIRCLEVAL           2                       //diff ofset for turning
    #define BASEHEIGHT          (-5)
    #define MOVEHEIGHT          3

    #define CALIBRATION         7                       //calibration interupt for 50Hz pwm
    #define INTERRUPTCOUNTER    (4000 - CALIBRATION)    //interupt counter for 50Hz pwm
    #define LIMITHI             500                     //limit for skip conditions after every 2.5ms
    
    #define PWMTIMEBASE         5                       //time base for interrupts
    #define DELAYTIMEBASE       500000                  //time base for delay interrupts
    #define DELAYCOUNTERMAXVAL  1                       //max value for delay_counter
       
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