/* 
  The encoder library is used to interface with hall sensors of 3 phase motors  
  A encoder is activated by creating an instance of the encoder class passing 
  the desired pins to the attach() method.

  Note that functionality of libraries and functions associated with the timers 5 will be affected when auto mode is used
  
  Automode updates the encoder in the background using interrupts at a rate of 25KHz. If time critical operations is required, automode is not recommended as the update_encoder() is relatively time consuming. 
  it is recommneded to set up an intterupt at the same rate and use flags to trigger update_encoder() for each encoder in the main loop instead.

  Regarding the scaling of encoder, as all motors have different poles and pulse per revolution, adjust the encoder_scaling until a satisfactory accuracty is achieved,
  Increase encoder_scaling to decrease the measured RPM vice versa. 

  The methods are:
    Encoder - Class for manipulating motors and controllers connected to Arduino pins.
    attach(Hall_A_pin, Hall_B_pin, Hall_C_pin, encoder_scaling);  - Attaches motor to the relavent i/o pins
    read()      - Gets the RPM of the motor. 
    update_encoder() - updates the encoder depending on the hall state 
 */

#ifndef encoder_h
#define encoder_h

//#define AUTO_MODE //comment to use manual mode

#include <Arduino.h>

class Encoder{
  public:
    encoder();
    void attach(int Hall_A_pin, int Hall_B_pin, int Hall_C_pin, double encoder_scaling); // attach the given pin to the encoder object
    void detach();
    void update_encoder(); //updates the encoder depending on the hall state
    int read_rpm(); //returns RPM of the motor
    void reset();
  private:
    int Hall_A_pin;
    int Hall_B_pin;
    int Hall_C_pin;    
    double encoder_scaling; 
    int Previous_Hall_state;
    long encoder_wheel;
    long prev_encoder_wheel;
    unsigned long encode_start_time; 
    int motor_index; //index of motor object on the hall interrupt ISR array     
};



#endif
