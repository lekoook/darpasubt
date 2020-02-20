#define USE_USBCON
#define NUM_SERVOS 3

#include <HCSR04.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#include <math.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include <Dropper.h>
#include <Servo.h>

void dropperCb(const vehicle_drive::Dropper& angles);

sensor_msgs::Range rangeMsg;
std_msgs::UInt16 co2Msg;
ros::NodeHandle nh;
ros::Publisher rangePub("ultrasonic", &rangeMsg);
ros::Publisher co2Pub("co2", &co2Msg);
ros::Subscriber<vehicle_drive::Dropper> droppersPub("droppers", dropperCb);
UltraSonicDistanceSensor distanceSensor(0, 1);
SCD30 airSensor;

Servo servosArr[NUM_SERVOS];
int servosPin[NUM_SERVOS] = {5, 6, 9};

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
    nh.advertise(co2Pub);
    nh.subscribe(droppersPub);
    Wire.begin();
    airSensor.begin();

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
    rangeMsg.range = dist;
    rangePub.publish(&rangeMsg);

    if (airSensor.dataAvailable())
    {
        co2Msg.data = airSensor.getCO2();
        co2Pub.publish(&co2Msg);
    }
    
    nh.spinOnce();
    delay(100);
}
