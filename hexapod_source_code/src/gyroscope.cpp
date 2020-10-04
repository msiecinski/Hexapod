#include <Gyroscope.hpp>

#define _DEBUG_

MPU6050 imu;     //MPU6050 object
double angle_x,angle_y;

#ifdef _DEBUG_
volatile unsigned long test = 0;
#endif

void GyroReadAngle(void)
{  
     /*
        Function using to read data from MPU6050 module
        next filtering by Kalman filter
    */
   
     int16_t ax, ay, az, gx, gy, gz;
    //double angle_x,angle_y;
    int sample;
   
    double ax_sum = 0;
    double ay_sum = 0;
    double az_sum = 0;
    double x,y,z;
    //add protection before no data from imu
    for(sample = 0; sample < NUMBERSAMPLES; )
    {
        if (imu.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz)) 
        {
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            sample++;
        }
    }
    x = ax_sum/NUMBERSAMPLES;
    y = ay_sum/NUMBERSAMPLES;
    z = az_sum/NUMBERSAMPLES;

    angle_x = -(atan2(x, sqrt(y*y + z*z))*180.0)/M_PI;
    angle_y = (atan2(y, z)*180.0)/M_PI;
#ifdef _DEBUG_
    Serial.print("Oś X:");
    Serial.print(angle_x);
    Serial.print(" ");
    Serial.print("Oś Y:");
    Serial.print(angle_y);
    Serial.print(" ");
    Serial.print("Test ");
    Serial.print(test);
    Serial.print(" ");
    Serial.println();
#endif
    ax_sum = 0;
    ay_sum = 0;
    az_sum = 0;
#ifdef _DEBUG_
    test++;
#endif
 }