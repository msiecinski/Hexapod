#include <Kinematics.hpp>



void InversKinematics(hexapod &newpos)
{
    /*
        Function InversKinematics takes on entry position(x,y,z),   
        next calculate and return necessary angles in degrees(q1,q2,q3)
    */
    double r,d;
	const double PI_value = 3.14159265358979323846; 

    newpos.angles.q1 = atan2(newpos.xyz.y,newpos.xyz.x);                                                            //q1

    r = sqrt(newpos.xyz.x * newpos.xyz.x+newpos.xyz.y * newpos.xyz.y);
	d = (((r-A1) * (r-A1) + (newpos.xyz.z-E) * (newpos.xyz.z-E) - (A2*A2)-(A3*A3)) / (2*A2*A3));
    newpos.angles.q3 = atan2(-sqrt(1-d), d);                                                                        //q3

	newpos.angles.q2 = atan2(newpos.xyz.z-E, r-A1) - atan2(A3*sin(newpos.angles.q3), A2+A3*cos(newpos.angles.q3));  //q2
    /*
        90 and 180 value based on set servos "0" posiotion 
    */
    newpos.angles.q1 *= (180 / PI_value);
    newpos.angles.q2 = (newpos.angles.q2*(180 / PI_value))+90;
    newpos.angles.q3 = (newpos.angles.q3 * (180 / PI_value))+180;
    
    AngleToDuty(newpos);
}

void AngleToDuty(hexapod &pos)
{
    /*
        Function AngleToDuty convert calculate angles to
        duty value for pwm singnal
        100=0.5ms 
        20/9-"equivalent" 0.45 degress
        if angle <0 - duty lovest work value duty = 100
        if >180 deg duty = 500
        else calculate
       // else  convert angle too duty and cut if>512(0x1FF)
    */
    pos.duty[0] = (pos.angles.q1<0) ? 100 : (pos.angles.q1>180) ? 500 : ((uint32_t)((pos.angles.q1*20/9)+100)); 
    pos.duty[1] = (pos.angles.q2<0) ? 100 : (pos.angles.q2>180) ? 500 : ((uint32_t)((pos.angles.q2*20/9)+100));
    pos.duty[2] = (pos.angles.q3<0) ? 100 : (pos.angles.q3>180) ? 500 : ((uint32_t)((pos.angles.q3*20/9)+100));
}

void RotateCordinate(const int leg, position &pos, const int offsetX, const int offsetY)
{
    /*
    Function using to rotate input offset
    fi value depends on chosen leg, 
    and base on robot mechanical build(servos base position)
    input data:
     y=0,x=offset, fi=0/45 or -45 deg,
    xcos(fi)-ycsin(fi)
    xsin(fi)+ycos(fi)
    */
    if(leg == 1 || leg == 4) //fi = 0 deg 
    {
        pos.x += offsetX;
        pos.y += offsetY;
    }
    else
    {
        if(leg == 2 || leg == 5)    //fi=+45 deg
        {
            pos.x += (double)((offsetX*0.70710678118) - (double)(offsetY*0.70710678118));    //xcos(fi)-ycsin(fi)
            pos.y += (double)((offsetX*0.70710678118) + (double)(offsetY*0.70710678118));     //xsin(fi)+ycos(fi)
        }
        else    // leg == 3 || leg == 0//fi=-45 deg
        {
            pos.x += (double)((offsetX*0.70710678118) + (double)(offsetY*0.70710678118)); 
            pos.y += (double)((-offsetX*0.70710678118) + (double)(offsetY*0.70710678118));;
        }
    }
}

double CalculateHeight(double angle, double length)
{
    /* 
        Funtion used to calculate height 
        needed to obtain the desired tilt
    */
    return(sqrt((2*length*length)-(2*length*length*cos(angle))));
}