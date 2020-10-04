#ifndef Kinematics_H_
#define Kinematics_H_

#include <Global_defines.hpp>
 
  //Functions designed to implement motion
  void InversKinematics(hexapod &);          //
  void AngleToDuty(hexapod &);              //conver input angles to duty for pwm
   
#endif /* Kinematics_H_ */