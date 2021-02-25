#ifndef Movement_H_
#define Movement_H_

#include <Global_defines.hpp>

_Bool SetPosition(int, const hexapod &);  //
_Bool SetLeg(int,const hexapod &);        
void SetThreeLegs(int,hexapod *);
void SetAllLegs(hexapod *);
void SlideHorizontal(int ,int );    //horizontal movement of the entire robot
void Move(movetype,int);            //direction,offset  
void MoveAtPlace(movetype,int);
void Control(uint16_t);
_Bool CheckGround(int);
void FindGround(int);
void PrepareWalk(movetype direction);

#endif /* Movement_H_ */