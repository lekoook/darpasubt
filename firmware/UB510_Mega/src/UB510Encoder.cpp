#include "UB510Encoder.hpp"

UB510Encoder::UB510Encoder(UB510Motor& motor) : motor(motor) {}

void UB510Encoder::attach(int pinPg, void (*userFunc)(void))
{
    this->pinPg = pinPg;
    attachInterrupt(digitalPinToInterrupt(pinPg), userFunc, RISING);
}

void UB510Encoder::updateTick()
{
    ticks++;
}

int UB510Encoder::getRpm()
{
    int rpm = previousRpm;
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;
    long currentTicks = ticks;
    long deltaTicks = currentTicks - previousTicks;
    
    // Check if ticks has overflowed. Reuse previous RPM if it has overflowed.
    if (deltaTicks < 1000000000 && deltaTicks > -1000000000)
    {
        double ticksFreq = deltaTicks / ((double)deltaTime / 1000.0f);
        rpm = RPM_CONSTANT * ticksFreq;
        rpm = (motor.getDirection() == MOTOR_FWD) ? rpm : -rpm;
    }

    previousTicks = currentTicks;
    previousTime = currentTime;
    previousRpm = rpm;
    return rpm;
}
