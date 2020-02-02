#include <Arduino.h>
#include <stdint.h>
//#include <Arduino_FreeRTOS.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>

#define YAW_PIN 13
#define SPEED_PIN 12
#define LEFT_WHEEL_ENCODER_PIN_FORWARD 2
#define LEFT_WHEEL_ENCODER_PIN_REVERSE 3
#define RIGHT_WHEEL_ENCODER_PIN_FORWARD 18
#define RIGHT_WHEEL_ENCODER_PIN_REVERSE 19
#define PWM_PERIOD 16540
#define MAX_SPEED 0.2
#define MAX_ANGULAR_SPEED 0.2
#define ROTATIONS_PER_CM 37


// Ugly looking macro for printing debug statements
char sprintfData[100];
#define PRINT_STR(str) do { \
        sprintf(sprintfData, str); \
        nh.loginfo(sprintfData); \
    } while (0)
#define PRINT(str, args) do { \
        sprintf(sprintfData, str, args); \
        nh.loginfo(sprintfData); \
    } while (0)

unsigned int yaw_delay = 1500;
unsigned int speed_delay = 1500;
bool stopped = true;

volatile int32_t leftWheelTicks = 0;
volatile int32_t leftWheelDistCm = 0;
volatile int32_t rightWheelTicks = 0;
volatile int32_t rightWheelDistCm = 0;

// Arduino setup and interrupt handling
void setupEncoderInterrupts();
void leftWheelEncoderInt();
void rightWheelEncoderInt();
void writePin();

//ROS Callbacks
void onTwistCmdRecieved(const geometry_msgs::Twist& twist);

//ROS stuff
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> commandSub("cmd_vel", &onTwistCmdRecieved);
std_msgs::Int16 leftWheelTicksMsg;
std_msgs::Int16 rightWheelTicksMsg;
ros::Publisher leftWheelOdomPub("lwheel", &leftWheelTicksMsg);
ros::Publisher rightWheelOdomPub("rwheel", &rightWheelTicksMsg);

void setup() {
  setupEncoderInterrupts();
  PRINT_STR("Initializing ROS Node....");
  nh.initNode();
  nh.advertise(leftWheelOdomPub);
  nh.advertise(rightWheelOdomPub);
  nh.subscribe(commandSub);
  PRINT_STR("Node Fully Initialized");
}

// ARDUINO INTERUPT SETUP AND HANDLING

void setupEncoderInterrupts() {
  pinMode(LEFT_WHEEL_ENCODER_PIN_FORWARD, INPUT);
  pinMode(RIGHT_WHEEL_ENCODER_PIN_FORWARD, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_ENCODER_PIN_FORWARD), leftWheelEncoderInt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_ENCODER_PIN_FORWARD), rightWheelEncoderInt, RISING);
}

inline void wheelEncoderHandler(int forwardPin, int reversePin, volatile int32_t* wheelTicks, 
                                      volatile int32_t* wheelDistance, int32_t rotationsPerCm) {
  bool isReverse = digitalRead(reversePin);
  if (isReverse) {
    (*wheelTicks)--;
  }
  else {
    (*wheelTicks)++;
  }
  
  if ((*wheelTicks) >= rotationsPerCm) {
    (*wheelTicks) = 0;
    (*wheelDistance) += 1;
  }
  else if ((*wheelTicks) <= -rotationsPerCm) {
    (*wheelTicks) = 0;
    (*wheelDistance) -= 1;
  }
}

void leftWheelEncoderInt() {
  wheelEncoderHandler(LEFT_WHEEL_ENCODER_PIN_FORWARD, LEFT_WHEEL_ENCODER_PIN_REVERSE,
                      &leftWheelTicks, &leftWheelDistCm, ROTATIONS_PER_CM);
}

void rightWheelEncoderInt()  {
  // weird x 2 rotations for right wheel???
  wheelEncoderHandler(RIGHT_WHEEL_ENCODER_PIN_FORWARD, RIGHT_WHEEL_ENCODER_PIN_REVERSE,
                      &rightWheelTicks, &rightWheelDistCm, 2*ROTATIONS_PER_CM);
}



// ROS Handling

void onTwistCmdRecieved(const geometry_msgs::Twist& twist) {
  if (twist.angular.z ==0 && twist.linear.x == 0) {
    stopped = true;
    return;
  }
  stopped = false;
  yaw_delay = 1500 - twist.angular.z/MAX_ANGULAR_SPEED*500;
  speed_delay = 1500 + twist.linear.x/MAX_SPEED*500;

  
  PRINT("Got Yaw Delay %d\n", yaw_delay);
  PRINT("Got Speed Delay %d\n", speed_delay);
}

void rosTask(void* pvParameters) {
  while(true) {
    nh.spinOnce();
    for(int i =0; i < 10; i++)
    writePin();

    leftWheelTicksMsg.data = leftWheelDistCm;
    rightWheelTicksMsg.data = rightWheelDistCm;
    
    leftWheelOdomPub.publish(&leftWheelTicksMsg);
    rightWheelOdomPub.publish(&rightWheelTicksMsg);
  }
}

void writePin() {
  if(stopped) {
    digitalWrite(YAW_PIN, LOW);
    digitalWrite(SPEED_PIN, LOW);
    return;
  }
  if (yaw_delay < speed_delay) {
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


void loop() {
  interrupts();
  rosTask(NULL);
  //vTaskStartScheduler();
 // other processing ... 
}