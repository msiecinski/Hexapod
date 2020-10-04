#ifndef Movement_H_
#define Movement_H_

#include <Global_defines.hpp>

_Bool SetPosition(int, hexapod &);  //
_Bool SetLeg(int,hexapod &);        
void SetLegs(int,hexapod *);
void SlideHorizontal(int ,int );    //horizontal movement of the entire robot
void Move(movetype,int);            //direction,offset  


#endif /* Movement_H_ */