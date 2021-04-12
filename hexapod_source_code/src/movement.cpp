#include <Movement.hpp>
#include <Kinematics.hpp>
#include <Gyroscope.hpp>


extern std::array<hexapod, 6> hexapodControl;
extern volatile int stepSize;
extern volatile uint32_t dutyBuff[6][3]; 

volatile uint32_t delayFlag = 0;

void SetPosition(int leg, const hexapod &pos)
{
    /*
        Function SetPositions used to calculate position selected robot leg
    */
    hexapodControl[leg].xyz = pos.xyz;          //copy position
    hexapodControl[leg].delay = pos.delay;      //copy delay
    hexapodControl[leg].offset = pos.offset;    //copy offset !!!!!!!!!!!!can be change
    hexapodControl[leg].update = 1;             //new values, so set flag!
    InversKinematics(hexapodControl[leg]);
}

void SetLeg(int leg, const hexapod &pos)
{
    return SetPosition(leg,pos);
}

void SetOneLeg(int leg, const hexapod &pos)
{
    SetLeg(leg,pos);
    while(!PushBuff());
    return;
}

void SetThreeLegs(int legNum,hexapod *pos)
{
    /*  Function is used to set position 
        three legs at the same time(1,3,5 or 2,4,6)
        legNum = 0 -set legs 1,3,5(tab[0],[2],[4])
        legNum = 1 - set legs 2,4,6(tab[1],[3],[5])
     */
    SetLeg((legNum+0),pos[0+legNum]);
    SetLeg((legNum+2),pos[2+legNum]);
    SetLeg((legNum+4),pos[4+legNum]);
    while(!PushBuff());
    return;
}

void SetAllLegs(hexapod *pos)
{
   
    /*  Function is used to set position 
        all legs @same time(if not delay performed)
    */
    SetLeg((0),pos[0]);
    SetLeg((1),pos[1]);
    SetLeg((2),pos[2]);
    SetLeg((3),pos[3]);
    SetLeg((4),pos[4]);
    SetLeg((5),pos[5]);
    while(!PushBuff());
    return;
}

_Bool PushBuff(void)
{
    if(delayFlag != 0)
        return FALSE;
    for(int i  = 0; i< 6; i++)
    {
        if(hexapodControl[i].update != 0)
        {
            for(int k =0; k <3; k++)
            {
                dutyBuff[i][k] = hexapodControl[i].duty[k];  
            }
            hexapodControl[i].update = 0;   //clear update flag
            if(hexapodControl[i].delay != 0)
            {
                delayFlag = hexapodControl[i].delay;  //should be clear via intterupt every x(?) ms!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
                hexapodControl[i].delay = 0; //clear flag in matrix(Kappa) 
            }
        }
    }
    return TRUE;
}

void SlideHorizontal(int movement_r,int movement_l)
{

    /* Function SlideHorizontal is used to 
        move horizontal entire surface of the robot
        sings plus/minus 1 can be changed !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    */

	int i ,j;
	hexapod  pos[6];
	_Bool SligerFlag = 1;

	for(int k = 0; k < 6; k++)
    {
        pos[k].xyz = hexapodControl[k].xyz; //copy actual position(xyz)
        pos[k].delay = 0;                   //clear all delay flags (for be sure)
        pos[k].offset = {0, 0, 0};          //clear all offsets (for be sure)
        pos[k].update = 0;                  //clear update flag
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
        RotateCordinate(0,pos[0].xyz,-i);
        RotateCordinate(1,pos[1].xyz,-i);
        RotateCordinate(2,pos[2].xyz,-i);
        RotateCordinate(3,pos[3].xyz,j);
        RotateCordinate(4,pos[4].xyz,j);
        RotateCordinate(5,pos[5].xyz,j);
        pos[5].delay = 5;               //after last leg always delay
        SetAllLegs(pos);
        if(0 == movement_r && 0 == movement_l)
            SligerFlag = 0;
    }
}

_Bool CheckGround(int leg)
{   
    /*
       Funtions using to read pins connect to limit switch
       each pin(limit switch) corresponds to one leg
       funtion return  TRUE(1) when "no ground"
       return "False(0)" when |"is ground"
       Debouncer is not nessesery bcs before every read is delay afer move
    */
    int SensorTab[] = { SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_5, SENSOR_6 };
    return((_Bool)digitalRead(SensorTab[leg]));
}

void FindGround(int leg)
{
    hexapod setPosition;
    int interration;
    setPosition.delay = VERTICALMOVEDELAY;              //set delay
    setPosition.xyz = hexapodControl[leg].xyz;          //copy position
    setPosition.offset = hexapodControl[leg].offset;    //copy offset
    setPosition.update = 0;                             //flag for update                  
    interration = MOVEHEIGHT + setPosition.offset.z;
    //First again move up leg
    for(int k = 1; k<=interration; k++)
    {
        setPosition.xyz.z = hexapodControl[leg].xyz.z + 1;
        setPosition.offset.z = hexapodControl[leg].offset.z - 1;
        SetOneLeg(leg,setPosition);
    }
}

void Move(movetype direction,int offset)
{
    /*
        Function  Move is used to implementation robot walk
        in set direction and distance
        This function is something like control interface of the robot
        1.Setting offset(depends on direction)
        2.Quantization //turn off 
        3.Leg 3(base position)
        TODO:
            Check ground
    */
    int posOffset[6];          //calculate offests depends on actual posiotn and move type(direction)
    int posOffsetTMP[6];
    hexapod setPosition[6];    //calculate step position for all legs
   
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
        setPosition[i].delay = 0;                   //init 0 value for delay 
        setPosition[i].update = 0;                   //init 0 value for update flag 
        setPosition[i].offset = {0,0,0};            //clear all offsets (for be sure)
        setPosition[i].xyz = hexapodControl[i].xyz; //copy actual position
        posOffsetTMP[i] = posOffset[i];             //copy value for nextinterration 
        if(i == 1 || i == 4)
            posOffset[i] = (-posOffset[i]);      //3 on ground 
    }
     /*  
        quantization of the movement
        first 3 legs move on thheground
        3 legs above the ground
    */
    for(int j = 0; j<2; j++)
    {
        //move up slowly 3 legs and add offest
        for(int k = 1; k<=MOVEHEIGHT; k++)
        {
            for(int i = j;  i<6; )
            {
                setPosition[i].xyz.z = hexapodControl[i].xyz.z+1;
                //add "old" offset to the "up" legs
                posOffset[0+i] += hexapodControl[0+i].offset.x; 
                i += 2;     //bcs 0-2-4 or 1-3-5 
            }
            setPosition[4+j].delay = VERTICALMOVEDELAY;
            SetThreeLegs(j,setPosition);
        }
        //now lets move
        for(int i = 0;  i<6; i++)
        { 
            RotateCordinate(i,setPosition[i].xyz,posOffset[i]);     //rotate offset
        }
        //set delay
        setPosition[5].delay = 1;
        SetAllLegs(setPosition);
        //move down slowly
        for(int k = 1; k<=MOVEHEIGHT; k++)
        {
            for(int i = j;  i<6; )
            {
                if(FALSE == CheckGround(i))
                {   //if "found" ground stop move down leg
                    hexapodControl[i].offset.z += 1;
                }
                else
                {
                    setPosition[i].xyz.z = hexapodControl[i].xyz.z-1;
                }
                i += 2;     //bcs 0-2-4 or 1-3-5 
            }
            setPosition[4+j].delay = VERTICALMOVEDELAY;
            SetThreeLegs(j,setPosition);
        }
        //switch offsets
        for(int i = 0; i<6; i++)
        {   
            if(i == 1 || i == 4)
                posOffset[i] = posOffsetTMP[i];
            else
                posOffset[i] = -posOffsetTMP[i];
        }
        /*for(int i = j; i<6; )
        {   //already olny move down Z if no leg
            hexapodControl[i].offset.x = 0;
            while(TRUE == CheckGround(i))
            {
                setPosition[i].xyz.z = hexapodControl[i].xyz.z-1;
                setPosition[i].delay = VERTICALMOVEDELAY; 
                hexapodControl[i].offset.z -= 1;
                SetOneLeg(i,setPosition[i]);
                if(hexapodControl[i].offset.z < -5)
                {
                    FindGround(); //still no ground so do something diffrent
                }  
            }
            i += 2;
        }*/
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
            setPosition[i].delay = 0;                       //clear all delay flags (for be sure)
            setPosition[i].offset = {0, 0, 0};              //clear all offsets (for be sure)
            setPosition[i].xyz.x = hexapodControl[i].xyz.x; //copy actual position(xyz)
            setPosition[i].update = 0;                      //flag for update
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
        setPosition[5].delay = VERTICALMOVEDELAY;
        SetAllLegs(setPosition);
    }
}

void Control(uint16_t data)
{
    /*
        Api function for control robot
    */
    uint8_t skipDelay = 0;      //variable using for skiping delays

    switch(data&0xFF)
    {
        case 0x1:
            Move(forvard,stepSize);
            break;
        case 0x2:
            Move(left,stepSize);
            break;
        case 0x3:
            Move(forvardleft,stepSize);
            break;
        case 0x4:
             Move(backward,stepSize);
            break;
        case 0x8:
            Move(right,stepSize);
            break;
        case 0x9:
            Move(forvardright,stepSize);
            break;
        case 0x10:
            MoveAtPlace(up,1);
            break;
        case 0x20:
            MoveAtPlace(wider,5);
            break;
        case 0x40:
            MoveAtPlace(narrower,5);
            break;
        case 0x80:
            MoveAtPlace(down,1);
            break;
        case 0x100:
            SlideHorizontal(-1,1);
            break;
        case 0x200:
            SlideHorizontal(1,-1);
            break;
        default:
            skipDelay++;    //no matching value on both switch statement = skipDelays
            break;  
    }
    
    switch((data>>8)&0xFF)
    {
        case 0x0:
            stepSize++;
            break;
        case 0x1:
            /*TODO*/
            //GyroControll(olny ps4 pad)
            break;
        case 0x2:
            /*TODO*/
            break;
        case 0x4:
            stepSize--;
            break;
        default:
            skipDelay++;    //no matching value on both switch statement = skipDelays
            break;
    }
    if(skipDelay != 2)
    {
        while(delayFlag);   //wait for clear after move
        delayFlag=1;        //set delay should be not too big
        while(delayFlag);
    }
}

void PrepareWalk(movetype direction)
{
    hexapod setPosition[6],tmp;

    for(int i = 0; i<6; i++)
    {   
        setPosition[i].delay = 0;                   //init 0 value for delay 
        setPosition[i].update = 0;                   //init 0 value for update flag 
        setPosition[i].offset = {0,0,0};            //clear all offsets (for be sure)
        setPosition[i].xyz = hexapodControl[i].xyz; //copy actual position
    }
    tmp.offset = {0,0,0};
    tmp.delay  = 0;

    for(int i = 0; i < 6; i++ )
    {
        tmp.xyz = {0,0,0};
        if(i == 1 || i == 4)
            continue;
        for(int k = 1; k<=MOVEHEIGHT; k++)
        {
            setPosition[i].xyz.z = hexapodControl[i].xyz.z+1;
            setPosition[i].delay = VERTICALMOVEDELAY;
            SetOneLeg(i,setPosition[i]);
        }
        if(i == 3 || i == 0)
        {
            if(direction == forvard)
            {
                setPosition[i].xyz = {0,19,hexapodControl[i].xyz.z};
            }
            else
            {
                RotateCordinate(i,tmp.xyz,0,19);
                setPosition[i].xyz = tmp.xyz;
            }
        }
        if(i == 2 || i == 5)
        {
            if(direction != forvard)
            {
                setPosition[i].xyz = {0,19,hexapodControl[i].xyz.z};
            }
            else
            {
                RotateCordinate(i,tmp.xyz,0,19);
                setPosition[i].xyz = tmp.xyz;
            }
        }
        setPosition[i].xyz.z = hexapodControl[i].xyz.z;
        setPosition[i].delay = VERTICALMOVEDELAY;
        SetOneLeg(i,setPosition[i]);
        
        for(int k = 1; k<=MOVEHEIGHT; k++)
        {
            setPosition[i].xyz.z = hexapodControl[i].xyz.z-1;
            setPosition[i].delay = VERTICALMOVEDELAY;
            SetOneLeg(i,setPosition[i]);
        }
    }  
}