#include "UB510Motor.hpp"
#include "UB510Encoder.hpp"
#include "TwistSolver.hpp"
#include "config.h"

UB510Motor motorA;
UB510Encoder encoderA(motorA);
UB510Motor motorB;
UB510Encoder encoderB(motorB);
UB510Motor motorC;
UB510Encoder encoderC(motorC);
UB510Motor motorD;
UB510Encoder encoderD(motorD);
BodyParams bodyParams(WHEEL_TO_WHEEL_DISTANCE, WHEEL_RADIUS);
TwistSolver twistSolver(bodyParams);

void updateA()
{
    encoderA.updateTick();
}

void updateB()
{
    encoderB.updateTick();
}

void updateC()
{
    encoderC.updateTick();
}

void updateD()
{
    encoderD.updateTick();
}

void setup()
{
    Serial.begin(115200);
    motorA.attach(MOTOR_A_SV, MOTOR_A_FR, MOTOR_A_EN, MOTOR_FWD);
    encoderA.attach(ENCODER_A_PG, updateA);
    motorB.attach(MOTOR_B_SV, MOTOR_B_FR, MOTOR_B_EN, MOTOR_BWD);
    encoderB.attach(ENCODER_B_PG, updateB);
    motorC.attach(MOTOR_C_SV, MOTOR_C_FR, MOTOR_C_EN, MOTOR_FWD);
    encoderC.attach(ENCODER_C_PG, updateC);
    motorD.attach(MOTOR_D_SV, MOTOR_D_FR, MOTOR_D_EN, MOTOR_BWD);
    encoderD.attach(ENCODER_D_PG, updateD);
    WheelAngular wheels;
    
    if (twistSolver.solve(0.2, 0, &wheels))
    {
        motorA.set((unsigned char)wheels.left, wheels.left >= 0 ? MOTOR_FWD : MOTOR_BWD);
        motorB.set((unsigned char)wheels.left, wheels.left >= 0 ? MOTOR_FWD : MOTOR_BWD);
        motorC.set((unsigned char)wheels.right, wheels.right >= 0 ? MOTOR_FWD : MOTOR_BWD);
        motorD.set((unsigned char)wheels.right, wheels.right >= 0 ? MOTOR_FWD : MOTOR_BWD);
    }
}


void loop()
{
  
}
