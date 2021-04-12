#ifndef Movement_H_
#define Movement_H_

#include <Global_defines.hpp>

void SetPosition(int, const hexapod &);  //
void SetLeg(int, const hexapod &);
void SetOneLeg(int, const hexapod &);           
void SetThreeLegs(int,hexapod *);
void SetAllLegs(hexapod *);
void SlideHorizontal(int ,int );    //horizontal movement of the entire robot
void Move(movetype,int);            //direction,offset  
void MoveAtPlace(movetype,int);
void Control(uint16_t);
_Bool CheckGround(int);
void FindGround(int);
void PrepareWalk(movetype direction);
_Bool PushBuff(void);

#endif /* Movement_H_ */