#define NUM_SERVOS 3

#include <HCSR04.h>
#include <math.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include <Servo.h>

UltraSonicDistanceSensor distanceSensor(11, 12);

//SCD30 airSensor;

Servo servosArr[NUM_SERVOS];
int servosPin[NUM_SERVOS] = {5, 6, 9};

void setup () {
//    Wire.begin();
    Serial.begin(9600);

//    airSensor.begin();

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
    Serial.println(dist);
/*
    if (airSensor.dataAvailable())
    {
	double co2 = airSensor.getCO2();
        Serial.println(co2);
    }
*/
    delay(100);
}
