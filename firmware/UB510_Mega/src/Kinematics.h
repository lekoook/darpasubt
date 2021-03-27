#ifndef KINEMATICS_H
#define KINEMATICS_H

#define TANGENTIAL_VEL_OFFSET 1
#define FORWARD false
#define REVERSE true
#include "Arduino.h"


class Kinematics
{
  public:
    struct motor {
      int req_rpm;
      byte pwm;
      bool dir;
    };  
    struct velocities {
      float linear_x;
      float angular_z;
    };
    motor motor_left;
    motor motor_right;
    velocities cur_vel;
    Kinematics();
    void set_up_Kinematics(int motor_max_rpm, float wheel_diameter, float wheels_x_distance, float gear_reduction, float left_speed_scale, float right_speed_scale);
    velocities updateVelocities(int cur_rpm_left, int cur_rpm_right);
    void pwm_calculate(float linear_x, float angular_z);
  private:
    int max_rpm;
    float wheel_diameter;
    float wheels_x_distance;
    float wheel_circumference;
    float gear_reduction;
    float left_speed_scale;
    float right_speed_scale;            
};

#endif
