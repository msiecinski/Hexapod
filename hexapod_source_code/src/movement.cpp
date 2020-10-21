#include <Movement.hpp>
#include <Kinematics.hpp>
#include <Gyroscope.hpp>

extern std::array<hexapod, 6> hexapodControl;
volatile uint32_t delayFlag = 0;



_Bool SetPosition(int leg, hexapod &pos)
{
    /*
        Function SetPositions used to set position selected robot leg
        If delayFlag = 1 function do nothing and return false(0)
        if delayFlag = 0 function set position and return true(1)
    */

    if(delayFlag != 0)
        return FALSE;
    hexapodControl[leg].xyz = pos.xyz;
     if(leg>2)
        hexapodControl[leg].xyz.x = -pos.xyz.x;
    hexapodControl[leg].delay = pos.delay;
    InversKinematics(hexapodControl[leg]);

    if(hexapodControl[leg].delay != 0)
    {
        delayFlag = hexapodControl[leg].delay;  //should be clear via intterupt every x(?) ms!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
        hexapodControl[leg].delay = 0; //clear flag in matrix(Kappa) 
    }
    return TRUE;
}

_Bool SetLeg(int leg,hexapod &pos)
{
     return SetPosition(leg,pos);
}

void SetThreeLegs(int legNum,hexapod *pos)
{
    /*  Function is used to set position 
        three legs at the same time(1,3,5 or 2,4,6)
        legNum = 0 -set legs 1,3,5(tab[0],[2],[4])
        legNum = 1 - set legs 2,4,6(tab[1],[3],[5])
     */
    //SetLeg((legNum+0),pos[0+legNum]);
    //SetLeg((legNum+2),pos[2+legNum]);
    while(!SetLeg((legNum+0),pos[0+legNum]));
    while(!SetLeg((legNum+2),pos[2+legNum]));
    while(!SetLeg((legNum+4),pos[4+legNum]));
}

void SetAllLegs(hexapod *pos)
{
   
    /*  Function is used to set position 
        all legs @same time(if not delay performed)
    */
    while(!SetLeg((0),pos[0]));
    while(!SetLeg((1),pos[1]));
    while(!SetLeg((2),pos[2]));
    while(!SetLeg((3),pos[3]));
    while(!SetLeg((4),pos[4]));
    while(!SetLeg((5),pos[5]));
}

void SlideHorizontal(int movement_r,int movement_l)
{

    /* Function SlideHorizontal is used to 
        move horizontal entire surface of the robot
        sings plus/minus 1 can be changed !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    */

	int i = 0;
	int j = 0;
    hexapod  pos[6];
	_Bool SligerFlag = 1;

	for(int k = 0; k < 6; k++)
    {
        pos[k].xyz = hexapodControl[k].xyz; //need change olny xyz.x value
        pos[k].delay = 0;                   //clear all delay flags (for be sure)
    }
                           
    while(SligerFlag)
    {   
        if(movement_r!=0)
        {
            if(movement_r>0)
            {
                j=1;
                movement_r--;
            }
            else 
            {
                j=-1;
                movement_r++;
            }
        }
        else
        {
                j=0;
        }
        if(movement_l!=0)
        {
            if(movement_l>0)
            {
                i=1;
                movement_l--;
            }
            else 
            {
                i=-1;
                movement_l++;
            }
        }
        else
        {
            i=0;
        }
        pos[0].xyz.x = hexapodControl[0].xyz.x - i;
        pos[1].xyz.x = hexapodControl[1].xyz.x - i;
        pos[2].xyz.x = hexapodControl[2].xyz.x - i;
        pos[3].xyz.x = hexapodControl[3].xyz.x + j;
        pos[4].xyz.x = hexapodControl[4].xyz.x + j;
        pos[5].xyz.x = hexapodControl[5].xyz.x + j;
        pos[5].delay = 5;               //after last leg always delay
        for(int k = 0; k < 6; k++)
        {
            while(!SetPosition(k,pos[k]));
        }
        if(0 == movement_r && 0 == movement_l)
            SligerFlag = 0;
    }
}

_Bool CheckGround(int leg)
{   
    return FALSE;
}

void Move(movetype direction,int offset)
{

    /*
        Function  Move is used to implementation robot walk
        in set direction and distance
        This function is something like control interface of the robot
        1.Setting offset(depends on direction)
        2.Quantization  
        3.Leg 3(base position)
        TODO:
            Check ground
        
    */
    int posOffset[6];          //calculate offests depends on actual posiotn and move type(direction)
    int posOffsetTMP[6];
    hexapod setPosition[6];    //calculate step position for all legs
    int dX[6];                //diff between base leg[2] and all another legs 
  
    switch(direction)
    {
       default:
       case forvard:
       {
           for(int i=0;i<6;i++)
                posOffset[i] = offset;
           break;
       }
       case backward:
       {    
           for(int i=0;i<6;i++)
                posOffset[i] = -offset;
            break;
       }
       case forvardleft:
       {
            for(int i=0;i<3;i++)
                posOffset[i] = offset;
            for(int i=3;i<6;i++)
                posOffset[i] = offset - CIRCLEVAL;
           break;
       }
       case forvardright:
       {
            for(int i=0;i<3;i++)
                posOffset[i] = offset - CIRCLEVAL;
            for(int i=3;i<6;i++)
                posOffset[i] = offset;
           break;
       }
         case left:
       {
            for(int i=0;i<3;i++)
                posOffset[i] = offset;
            for(int i=3;i<6;i++)
                posOffset[i] = -offset;
           break;
       }
       case right:
       {
            for(int i=0;i<3;i++)
                posOffset[i] = -offset;
            for(int i=3;i<6;i++)
                posOffset[i] = offset;
           break;
       }
   }
    
    for(int i = 0; i<6; i++)
    {   
        //use for compress diff position (maybe bcs scan floor)
        dX[i] = hexapodControl[i].xyz.x - hexapodControl[2].xyz.x;  
        posOffset[i] -= dX[i];              
        posOffsetTMP[i] = posOffset[i];         //copy value for nextinterration
        if(i%2)
            posOffset[i] *= (-posOffset[2]);      //3 on ground 
        
    }
    /*  
        quantization of the movement
        first 3 legs move on thheground
        3 legs above the ground
    */
    for(int j=0; j<2; j++)
    {
        //move up 3 legs
        for(int i = j;  i<6; )
        {
            setPosition[i].xyz.x = hexapodControl[i].xyz.x;
            setPosition[i].xyz.y = hexapodControl[i].xyz.y;
            setPosition[i].xyz.z = hexapodControl[i].xyz.z+MOVEHEIGHT;//////!!!!!!!!!!!we will see value @future
            i += 2;     //bcs 0-2-4 or 1-3-5 
        }
        setPosition[4+j].delay = 1;
        SetThreeLegs(j,setPosition);

        //now lets move
        while(posOffset[0+j] || posOffset[2+j] || posOffset[4+j])
        {
            for(int i = 0;  i<6; i++)
            {
                if(posOffset[i])
                {
                    setPosition[i].xyz.x = hexapodControl[i].xyz.x + 1;
                    posOffset[i]--; 
                }
                else
                {
                    if(posOffset[i]<0)
                    {
                        setPosition[i].xyz.x = hexapodControl[i].xyz.x - 1;
                        posOffset[i]++; 
                    }
                    else  
                        setPosition[i].xyz.x = hexapodControl[i].xyz.x;  //do nothing untill all posOffset[i]!-=0;
                }
                setPosition[i].xyz.y = hexapodControl[i].xyz.y;
                setPosition[i].xyz.z = hexapodControl[i].xyz.z;
           }
            //set delay
            setPosition[5].delay = 1;
            SetAllLegs(setPosition);
        }
        //move down slowly
        for(int k=1; k<=MOVEHEIGHT; k++)
        {
            for(int i = j;  i<6; )
            {
                setPosition[i].xyz.x = hexapodControl[i].xyz.x;
                setPosition[i].xyz.y = hexapodControl[i].xyz.y;
                setPosition[i].xyz.z = hexapodControl[i].xyz.z-k;
                i += 2;     //bcs 0-2-4 or 1-3-5 
            }
            //CheckGround() here is needed !!!!!!!!!!!!!!!!!
            setPosition[4+j].delay = 1;
            SetThreeLegs(j,setPosition);
        }
         for(int i = 0; i<6; i++)
        {   
            if(i%2)
                posOffset[i] = posOffsetTMP[i];
            else
                posOffset[i] = posOffsetTMP[2];
        }
    }
    //TODO:check the ground!!!!!!!!!!!!!!!!!!
}

void TestSuppFunction(int legNum,int *posOffset,hexapod *setPosition)
{
     for(int i = legNum;  i<6;    )
            {
                if(posOffset[i])
                {
                    setPosition[i].xyz.x = hexapodControl[i].xyz.x + 1;
                    posOffset[i]--; 
                }
                else
                {
                    if(posOffset[i]<0)
                    {
                        setPosition[i].xyz.x = hexapodControl[i].xyz.x - 1;
                        posOffset[i]++; 
                    }
                    else  
                        setPosition[i].xyz.x = hexapodControl[i].xyz.x;  //do nothing untill all posOffset[i]!-=0;
                }
                setPosition[i].xyz.y = hexapodControl[i].xyz.y;
                setPosition[i].xyz.z = BASEHEIGHT+MOVEHEIGHT;
                i += 2;     //bcs 0-2-4 or 1-3-5
            }   
}

void MoveAtPlace(movetype direction,int offset)
{
    hexapod setPosition[6];
    int moveOffset;
    _Bool axis;

    switch(direction)
    {
        case up:
            moveOffset = 1;
            axis = 0;
            break;
        case down:
            moveOffset = -1;
            axis = 0;
            break;
        case wider:
            moveOffset = 1;
            axis = 1;
            break;
        case narrower:
            moveOffset = -1;
            axis = 1;
            break;
        default:
            moveOffset = 0;
            axis=0;
            break;
    }
    for(int k=0; k<=offset; k++)
    {
        for(int i = 0;  i<6; i++ )
        {
            setPosition[i].xyz.x = hexapodControl[i].xyz.x;
            if(axis == 0)
            {
                setPosition[i].xyz.y = hexapodControl[i].xyz.y;
                setPosition[i].xyz.z = hexapodControl[i].xyz.z + (moveOffset);
            }
            else
            {
                setPosition[i].xyz.y = hexapodControl[i].xyz.y + (moveOffset);
                setPosition[i].xyz.z = hexapodControl[i].xyz.z;
            }
        }
        setPosition[5].delay = 1;
        SetAllLegs(setPosition);
    }
}
