#include <Arduino.h>
//#include <Arduino_FreeRTOS.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>

#define YAW_PIN 13
#define SPEED_PIN 12
#define PWM_PERIOD 16540
#define MAX_SPEED 0.2
#define MAX_ANGULAR_SPEED 0.2

unsigned int yaw_delay = 1500;
unsigned int speed_delay = 1500;
bool stopped = true;
void writePin();

//ROS Callbacks
void onTwistCmdRecieved(const geometry_msgs::Twist& twist);

//ROS stuff
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwistCmdRecieved);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  
}

void onTwistCmdRecieved(const geometry_msgs::Twist& twist)
{
  if(twist.angular.z ==0 && twist.linear.x == 0) {
    stopped = true;
    return;
  }
  stopped = false;
  yaw_delay = 1500 - twist.angular.z/MAX_ANGULAR_SPEED*500;
  speed_delay = 1500 + twist.linear.x/MAX_SPEED*500;
  char data[100];
  sprintf(data, "Got Yaw Delay %d\n", yaw_delay);
  nh.loginfo(data);
  sprintf(data, "Got Speed Delay %d\n", speed_delay);
    nh.loginfo(data);

}

void rosTask(void* pvParameters)
{
  while(true) {
    nh.spinOnce();
    for(int i =0; i < 10; i++)
    writePin();
  }
}
void writePin() {
  if(stopped){
    digitalWrite(YAW_PIN, LOW);
    digitalWrite(SPEED_PIN, LOW);
    return;
  }
  if(yaw_delay < speed_delay) {
    digitalWrite(YAW_PIN, HIGH);
    digitalWrite(SPEED_PIN, HIGH);
    delayMicroseconds(yaw_delay);
    digitalWrite(YAW_PIN, LOW);
    delayMicroseconds(speed_delay-yaw_delay);
    digitalWrite(SPEED_PIN, LOW);
    delayMicroseconds(PWM_PERIOD - speed_delay);
  }
  else{
    digitalWrite(YAW_PIN, HIGH);
    digitalWrite(SPEED_PIN, HIGH);
    delayMicroseconds(speed_delay);
    digitalWrite(SPEED_PIN, LOW);
    delayMicroseconds(yaw_delay-speed_delay);
    digitalWrite(YAW_PIN, LOW);
    delayMicroseconds(PWM_PERIOD - yaw_delay);
  }
}
void loop()
{
  rosTask(NULL);
  //vTaskStartScheduler();
 // other processing ... 
}