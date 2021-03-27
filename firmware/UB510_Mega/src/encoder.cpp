#include "encoder.h"
#include "config.h"
#include <Arduino.h>

Encoder* attached_encoder[6];
int Attached_No = 0;


static void init_ISR(int sv_pin)
{
  cli();//stop interrupts

  #ifdef AUTO_MODE
  //set up hall_sensor interrupt // timer5
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;//initialize counter value to 0
  // set compare match register for 2khz intervals
  OCR5A = 10;//124;
  // turn on CTC mode
  TCCR5B |= (1 << WGM52);
  // Set CS50 and CS51 bits for 64 prescaler
  TCCR5B |= (1 << CS51) | (1 << CS50);  
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE5A);
  #endif
  
  sei();//allow interrupts
}

#ifdef AUTO_MODE
ISR(TIMER5_COMPA_vect) //timer5 interrupt 2kHz
{
  for (int idx = 0; idx < Attached_No; idx++) 
  {
    attached_motor[idx]->update_encoder();
  }
}
#endif

Encoder::encoder() 
{
   
}

void Encoder::attach(int Hall_A_pin, int Hall_B_pin, int Hall_C_pin, double encoder_scaling)
{
  this->Hall_A_pin = Hall_A_pin;
  this->Hall_B_pin = Hall_B_pin;
  this->Hall_C_pin = Hall_C_pin;

  this->encoder_scaling = encoder_scaling;

  this->Previous_Hall_state = 0;
  this->encoder_wheel = 0;
  this->prev_encoder_wheel = 0;
  this->encode_start_time = millis();

  #ifdef auto_mode
  this->motor_index = Attached_No;
  attached_motor[this->motor_index] = this;
  Attached_No++;
  #endif

  pinMode( Hall_A_pin, INPUT);
  pinMode( Hall_B_pin, INPUT);
  pinMode( Hall_C_pin, INPUT);   
}

void Encoder::detach()
{
  #ifdef AUTO_MODE
  Attached_No--;
  attached_motor[this->motor_index] = attached_motor[Attached_No];
  attached_motor[this->motor_index]->motor_index = this->motor_index;
  #endif
}

void Encoder::update_encoder()
{
  bool Hall_A, Hall_B, Hall_C;
  int Current_Hall_state = 0;
  Hall_A = digitalRead(this->Hall_A_pin);
  Hall_B = digitalRead(this->Hall_B_pin);
  Hall_C = digitalRead(this->Hall_C_pin);
  Current_Hall_state = ( Hall_A << 2) | ( Hall_B << 1) | ( Hall_C << 0);
  
  switch (Current_Hall_state) {
  case 1:
    if (this->Previous_Hall_state == 5) {
      encoder_wheel++;
    } else if (this->Previous_Hall_state == 3) {
      encoder_wheel--;
    }
    break;
  case 3:
    if (this->Previous_Hall_state == 1) {
      encoder_wheel++;
    } else if (this->Previous_Hall_state == 2) {
      encoder_wheel--;
    }
    break;
  case 2:
    if (this->Previous_Hall_state == 3) {
      encoder_wheel++;
    } else if (this->Previous_Hall_state == 6) {
      encoder_wheel--;
    }
    break;
  case 6:
    if (this->Previous_Hall_state == 2) {
      encoder_wheel++;
    } else if (this->Previous_Hall_state == 4) {
      encoder_wheel--;
    }
    break;
  case 4:
    if (this->Previous_Hall_state == 6) {
      encoder_wheel++;
    } else if (this->Previous_Hall_state == 5) {
      encoder_wheel--;
    }
    break;
  case 5:
    if (this->Previous_Hall_state == 4) {
      encoder_wheel++;
    } else if (this->Previous_Hall_state == 1) {
      encoder_wheel--;
    }
    break;
  default:
    //Serial.println( "hall sensor error");
    break;
  }

  Previous_Hall_state = Current_Hall_state;
}

int Encoder::read_rpm()
{
  unsigned long current_timer;
  current_timer= millis();
  unsigned long dt = current_timer - encode_start_time;
  double dtm = (double)dt / 60000.0;
  double delta_ticks = (double)encoder_wheel - (double)prev_encoder_wheel;
  double ret = ((delta_ticks / 31.0) /(double) dtm); 
  ret /= encoder_scaling; //scale to motor  
  encode_start_time = current_timer;
  prev_encoder_wheel = encoder_wheel;
  return ret;
}

void Encoder::reset()
{
  this->Previous_Hall_state = 0;
  this->encoder_wheel = 0;
  this->prev_encoder_wheel = 0;
  this->encode_start_time = millis();  
}

  
