#ifndef H_UB510_ENCODER
#define H_UB510_ENCODER

#include <Arduino.h>
#include "UB510Motor.hpp"

/**
 * @brief Number of pole pairs. Actual number of poles will be twice this number.
 * 
 */
#define POLE_PAIRS 2

/**
 * @brief Represents a single encoder object connected to the UB510 controller.
 * 
 */
class UB510Encoder
{
private:
    /**
     * @brief Constant used to multiply to the tick frequency.
     * RPM = (20 * tick frequency) / (number of poles)
     * 
     */
    double const RPM_CONSTANT =  20.0f / (POLE_PAIRS * 2.0f);

    /**
     * @brief UB510Motor that will be used to read the direction of motor.
     * 
     */
    UB510Motor& motor;

    /**
     * @brief Pin that is connected to PG output on UB510.
     * 
     */
    int pinPg = 0;

    /**
     * @brief Previous tick count.
     * 
     */
    volatile long previousTicks = 0;

    /**
     * @brief Running count of ticks.
     * 
     */
    volatile long ticks = 0;

    /**
     * @brief Previous timestamp used to get RPM.
     * 
     */
    volatile unsigned long previousTime = 0;

    /**
     * @brief The last RPM calculated.
     * 
     */
    volatile int previousRpm = 0;

public:
    /**
     * @brief Construct a new UB510Encoder object.
     * 
     * @param motor UB510Motor object.
     */
    UB510Encoder(UB510Motor& motor);

    /**
     * @brief Attaches the encoder to the pin outputs of UB510 controller.
     * 
     * @param pinPg Pin that is connected to PG output on UB510.
     */

    /**
     * @brief Attaches the encoder to the pin outputs of UB510 controller.
     * 
     * @param pinPg Pin that is connected to PG output on UB510.
     * @param userFunc Function that will be called whenever the PG signal goes from LOW to HIGH.
     */
    void attach(int pinPg, void (*userFunc)(void));

    /**
     * @brief Updates the encoder tick. Should be called each time a tick occurs.
     * 
     */
    void updateTick();

    /**
     * @brief Gets the current Revolutions Per Minute (RPM) of the encoder.
     * 
     * @return int RPM of encoder.
     */
    int getRpm();
};

#endif // H_UB510_ENCODER