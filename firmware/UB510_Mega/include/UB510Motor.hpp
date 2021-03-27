#ifndef H_UB510_MOTOR
#define H_UB510_MOTOR

#include <Arduino.h>

/**
 * @brief Denotes the forward direction for motor.
 * 
 */
#define MOTOR_FWD true

/**
 * @brief Denotes the backward direction for motor.
 * 
 */
#define MOTOR_BWD false

/**
 * @brief Represents a single motor object connected to the UB510 controller.
 * 
 */
class UB510Motor
{
private:
    /**
     * @brief Pin that is connected to SV input on UB510.
     * 
     */
    int pinSv = 0;

    /**
     * @brief Pin that is connected to FR input on UB510.
     * 
     */
    int pinFr = 0;

    /**
     * @brief Pin that is connected to EN input on UB510.
     * 
     */
    int pinEn = 0;

    /**
     * @brief Orientation of the motor. Takes the value of either MOTOR_FWD or MOTOR_BWD. If MOTOR_FWD, the motor shaft 
     * should be facing RIGHT when looking in the forward direction. Vice versa.
     * 
     */
    bool orientation = MOTOR_FWD;

    /**
     * @brief Direction the motor is currently spinning. Takes the value of either MOTOR_FWD or MOTOR_BWD.
     * 
     */
    bool direction = MOTOR_FWD;

public:
    /**
     * @brief Construct a new UB510Motor object.
     * 
     */
    UB510Motor() = default;
    
    /**
     * @brief Attaches the motor to the pin inputs of UB510 controller.
     * 
     * @param pinSv Pin that is connected to SV input on UB510.
     * @param pinFr Pin that is connected to FR input on UB510.
     * @param pinEn Pin that is connected to EN input on UB510.
     * @param orientation Orientation of the motor. Takes the value of either MOTOR_FWD or MOTOR_BWD. 
     * If MOTOR_FWD, the motor shaft should be facing RIGHT when looking in the forward direction. Vice versa.
     */
    void attach(int pinSv, int pinFr, int pinEn, bool orientation);

    /**
     * @brief Detach the motor. Undefined behaviour if the motor was not attached already.
     * 
     */
    void detach();

    /**
     * @brief Sets the PWM value and direction for the motor.
     * 
     * @param pwm PWM value. Takes a value within the range of [0,255].
     * @param direction Direction of motor to drive. Takes the value of either MOTOR_FWD or MOTOR_BWD.
     */
    void set(unsigned char pwm, bool direction);

    /**
     * @brief Sets the PWM value for the motor.
     * 
     * @param pwm PWM value. Takes a value within the range of [0,255].
     */
    void set(unsigned char pwm);

    /**
     * @brief Sets the direction for the motor.
     * 
     * @param direction Direction of motor to drive. Takes the value of either MOTOR_FWD or MOTOR_BWD.
     */
    void set(bool direction);

    /**
     * @brief Gets the direction for the motor.
     * 
     * @return MOTOR_FWD Forward direction. 
     * @return MOTOR_BWD Backward direction.
     */
    bool getDirection();

    /**
     * @brief Enables the operation of motor.
     * 
     */
    void enable();

    /**
     * @brief Disables the operation of motor.
     * 
     */
    void disable();
};

#endif // H_UB510_MOTOR