/* 
 This library is made to interface atmega chip to BLS510 bldc controllers
  A controller is activated by creating an instance of the bls510 class and passing 
  the desired pins to the attach() method.

  Note that functionality of libraries and functions associated with the timers below may be affected when the corresponding pins are first attached to a motor.
  Timer 1 : Pin 11, 12 and 13
  Timer 3 : Pin 2, 3 and 5
  Timer 4 : Pin 6, 7 and 8
  Timer 5 : Pins 2, 3, 5, 6, 7, 8, 11, 12 and 13

  The SV pins will only work on the following pins 2, 3, 5, 6, 7, 8, 11, 12 and 13
  

  The methods are:
    BLS510 - Class for manipulating controllers connected to Arduino pins.
    attach(sv_pin, fr_pin, en_pin, Hall_A_pin, Hall_B_pin, Hall_C_pin, Orientation);  - Attaches the bls510 controller to the relavent i/o pins.
    write(PWM, dir)     -  Runs the motor attached to relavent pwm and direction
    attached()  - Returns true if there is a motor attached. 
    detach()    - Stops an attached motor from pulsing its i/o pins. 
 */

#ifndef bls510_h
#define bls510_h

#include <Arduino.h>

#define MOTOR_FORWARD         true
#define MOTOR_REVERSE         false

class BLS510{
  public:
    BLS510();
    void attach(int sv_pin, int fr_pin, int en_pin, bool Orientation); // attach the given pin to the controller object
    void detach(); 
    void write(byte pwm, bool dir); //generate PWM for the respective controller    
    bool attached(); //return true if controller is attached
    void disable();
    void enable();
  private:
    int sv_pin;
    int fr_pin;
    int en_pin;
    bool Orientation;    
};



#endif
