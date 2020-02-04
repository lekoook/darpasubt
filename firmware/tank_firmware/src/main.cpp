#include <Arduino.h>
#include <stdint.h>
//#include <Arduino_FreeRTOS.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>

#define YAW_PIN 13
#define SPEED_PIN 12
#define LEFT_WHEEL_ENCODER_PIN_FORWARD 2
#define LEFT_WHEEL_ENCODER_PIN_REVERSE 3
#define RIGHT_WHEEL_ENCODER_PIN_FORWARD 18
#define RIGHT_WHEEL_ENCODER_PIN_REVERSE 19
#define PWM_PERIOD 16540
#define MAX_SPEED 0.1
#define MAX_ANGULAR_SPEED 0.2
#define ROTATIONS_PER_CM 37

#define BOTTOM_SONAR_TRIG 20
#define BOTTOM_SONAR_ECHO 21

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
sensor_msgs::Range frontSonarMsg;
sensor_msgs::Range bottomSonarMsg;
ros::Publisher leftWheelOdomPub("lwheel", &leftWheelTicksMsg);
ros::Publisher rightWheelOdomPub("rwheel", &rightWheelTicksMsg);
ros::Publisher bottomSonarPub("ultrasound/bottom", &bottomSonarMsg);

void setup() {
  setupEncoderInterrupts();
  PRINT_STR("Initializing ROS Node....");
  nh.initNode();
  nh.advertise(leftWheelOdomPub);
  nh.advertise(rightWheelOdomPub);
  nh.advertise(frontSonarPub);
  nh.advertise(bottomSonarPub);
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

inline long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

long readPingSonar(int pingPin) {
  long duration;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}

long readHC04Sonar(int trigPin, int echoPin) {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  return microsecondsToCentimeters(duration);
}
// ROS Handling

void onTwistCmdRecieved(const geometry_msgs::Twist& twist) {
  if (twist.angular.z ==0 && twist.linear.x == 0) {
    stopped = true;
    return;
  }
  stopped = false;
  yaw_delay = 1500 - twist.angular.z/MAX_ANGULAR_SPEED*500;
  speed_delay = 1500 - twist.linear.x/MAX_SPEED*500;

  
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
    bottomSonarMsg.header.frame_id = "BOTTOM_SONAR";
    bottomSonarMsg.header.stamp = nh.now();
    bottomSonarMsg.range = 0.01f*(float)readHC04Sonar(BOTTOM_SONAR_TRIG, BOTTOM_SONAR_ECHO);
    bottomSonarMsg.min_range = 0.02;
    bottomSonarMsg.max_range = 4;
    bottomSonarPub.publish(&frontSonarMsg);
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