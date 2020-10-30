#include <Gyroscope.hpp>

MPU6050 imu;     //MPU6050 object
double angle_x, angle_y;

void GyroReadAngle(void)
{  
     /*
        Function using to read data from MPU6050 module
        next filtering by Kalman filter
    */
    int16_t ax, ay, az, gx, gy, gz;
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
    ax_sum = 0;
    ay_sum = 0;
    az_sum = 0;
}