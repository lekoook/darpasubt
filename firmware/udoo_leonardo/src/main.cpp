// Check which board is used. Leonardo or Teensy
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#define USING_LEONARDO
#endif
#ifdef CORE_TEENSY
#define USING_TEENSY
#endif


#ifdef USING_LEONARDO
#define D_PIN_1 5
#define D_PIN_2 6
#define D_PIN_3 9
#define USE_USBCON
#include <Servo.h>
#endif


#ifdef USING_TEENSY
#define D_PIN_1 3
#define D_PIN_2 4
#define D_PIN_3 5
#include <PWMServo.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#endif


#define NUM_SERVOS 3

#include <HCSR04.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Dropper.h>

// Function prototypes
void dropperCb(const vehicle_drive::Dropper& angles);

// Common global declarations for all boards
std_msgs::Float32 rangeMsg;
ros::NodeHandle nh;
ros::Publisher rangePub("ultrasonic", &rangeMsg);
ros::Subscriber<vehicle_drive::Dropper> droppersPub("droppers", dropperCb);

// Global declaration for Leonardo board.
#ifdef USING_LEONARDO
Servo servosArr[NUM_SERVOS];
UltraSonicDistanceSensor distanceSensor(11, 12);
#endif

// Global declaration for Teensy board.
#ifdef USING_TEENSY
PWMServo servosArr[NUM_SERVOS];
std_msgs::UInt16 co2Msg;
ros::Publisher co2Pub("co2", &co2Msg);
SCD30 airSensor;
UltraSonicDistanceSensor distanceSensor(9, 10);
#endif

void dropperCb(const vehicle_drive::Dropper& angles)
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
	servosArr[i].write(angles.dropper_angles[i]);
    }
}

void setup () {
    nh.initNode();
    nh.advertise(rangePub);
    nh.subscribe(droppersPub);

    #ifdef USING_TEENSY
    nh.advertise(co2Pub);
    Wire.begin();
    airSensor.begin();
    #endif

    int servosPin[NUM_SERVOS] = {D_PIN_1, D_PIN_2, D_PIN_3};

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        servosArr[i].attach(servosPin[i]);
        servosArr[i].write(60);
    }

}

void loop () {

    double dist = distanceSensor.measureDistanceCm();
    if (dist == -1.0)
    {
        dist = INFINITY;
    }
    else
    {
        dist = dist / 100.0;
    }
    rangeMsg.data = dist;
    rangePub.publish(&rangeMsg);

    #ifdef USING_TEENSY
    if (airSensor.dataAvailable())
    {
	    double co2 = airSensor.getCO2();
        co2Msg.data = co2;
        co2Pub.publish(&co2Msg);
    }
    #endif

    nh.spinOnce();
    delay(100);
}
