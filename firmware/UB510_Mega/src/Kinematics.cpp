#include "Kinematics.h"

#define constrain(amt,low,high) \
	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

Kinematics::Kinematics()
{
  
}

void Kinematics::set_up_Kinematics(int motor_max_rpm, float wheel_diameter, float wheels_x_distance, float gear_reduction, float left_speed_scale, float right_speed_scale)
{
  this->wheel_diameter = wheel_diameter;
  this->wheel_circumference = PI * wheel_diameter;
  this->max_rpm = motor_max_rpm;
  this->wheels_x_distance = wheels_x_distance;
  this->gear_reduction = gear_reduction;
  this->left_speed_scale = left_speed_scale;  
  this->right_speed_scale = right_speed_scale;

  this->motor_left.req_rpm = 0;
  this->motor_left.pwm = 0;
  this->motor_right.req_rpm = 0;
  this->motor_right.pwm = 0;
}

void Kinematics::pwm_calculate(float linear_x, float angular_z)
{
  float linear_vel_x_mins;
  float angular_vel_z_mins;
  float tangential_vel;
  float x_rpm;
  float y_rpm;
  float tan_rpm;
  
  //convert m/s to m/min
  linear_vel_x_mins = linear_x * 60;    
  
  //convert rad/s to rad/min
  angular_vel_z_mins = angular_z * 60;
  
  tangential_vel = TANGENTIAL_VEL_OFFSET * (angular_vel_z_mins * (this->wheels_x_distance / 2));
  
  
  x_rpm = linear_vel_x_mins / wheel_circumference;
  tan_rpm = tangential_vel / wheel_circumference;
  
  //calculate for the target motor RPM and direction
  
  //left motor
  this->motor_left.req_rpm = x_rpm - tan_rpm;
  this->motor_left.req_rpm *= gear_reduction;
  this->motor_left.req_rpm *= left_speed_scale;
  this->motor_left.req_rpm = constrain(this->motor_left.req_rpm, -max_rpm, max_rpm);
  //right motor
  this->motor_right.req_rpm = x_rpm + tan_rpm;
  this->motor_right.req_rpm *= gear_reduction;
  this->motor_right.req_rpm *= right_speed_scale; 
  this->motor_right.req_rpm = constrain(this->motor_right.req_rpm, -max_rpm, max_rpm);

  //calculate required PWM
  
  //left motor
  if (this->motor_left.req_rpm < 0 )
  {
    motor_left.req_rpm *= -1;
    this->motor_left.dir = FORWARD;
  }
  else
  {
     this->motor_left.dir = REVERSE;
  }
  this->motor_left.pwm = this->motor_left.req_rpm*255L/max_rpm;
  
  //right motor //mounted other way
  if (this->motor_right.req_rpm < 0 )
  {
    this->motor_right.req_rpm *= -1;
    this->motor_right.dir = REVERSE;
  }
  else
  {
    this->motor_right.dir = FORWARD;
  }  
  this->motor_right.pwm = this->motor_right.req_rpm*255L/max_rpm;  
}

Kinematics::velocities Kinematics::updateVelocities(int cur_rpm_left, int cur_rpm_right)
{
  float average_rps_x;
  float average_rps_a;  

  //convert average revolutions per minute to revolutions per second
  average_rps_x = ((float)(cur_rpm_left + cur_rpm_right) / 2) / 60; // RPM
  this->cur_vel.linear_x = average_rps_x * this->wheel_circumference; // m/s

  //convert average revolutions per minute to revolutions per second
  average_rps_a = ((float)(-cur_rpm_left + cur_rpm_right) / 20) / 60;
  this->cur_vel.angular_z =  (average_rps_a * wheel_circumference) / (wheels_x_distance / 2); //  rad/s
  
  return this->cur_vel;
}




























 
