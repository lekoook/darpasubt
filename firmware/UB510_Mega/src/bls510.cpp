#include "bls510.h"
#include "config.h"
#include <Arduino.h>

static void init_ISR(int sv_pin)
{
  cli();//stop interrupts
  //set up pwm interrupt 
  switch(sv_pin) 
  {
    case 2: 
    case 3:
    case 5:
      // timer 3
      TCNT3  = 0;
      TCCR3A &= ~((1 << WGM30)|(1 << WGM31)); //reset timer settings
      TCCR3A |= (1 << WGM31); //set up phase correct pwm
      TCCR3B &= ~((1 << WGM33)|(1 << WGM32)|(1 << CS32)|(1 << CS31)|(1 << CS30)); //reset timer settings
      TCCR3B |= (1 << WGM33)|(1 << CS31); //set up phase correct counter and counter for 8 prescaler
      ICR3H = 0;
      ICR3L = 255;
      break;        
    case 6:   
    case 7: 
    case 8:
      // timer 4
      TCNT4  = 0;
      TCCR4A &= ~((1 << WGM40)|(1 << WGM41)); //reset timer settings
      TCCR4A |= (1 << WGM41); //set up phase correct pwm
      TCCR4B &= ~((1 << WGM43)|(1 << WGM42)|(1 << CS42)|(1 << CS41)|(1 << CS40)); //reset timer settings
      TCCR4B |= (1 << WGM43)|(1 << CS41); //set up phase correct counter and counter for 8 prescaler
      ICR4H = 0;
      ICR4L = 255;      
      break;   
    case 11:   
    case 12:
    case 13:
      // timer 1
      TCNT1  = 0;
      TCCR1A &= ~((1 << WGM10)|(1 << WGM11)); //reset timer settings
      TCCR1A |= (1 << WGM11); //set up phase correct pwm
      TCCR1B &= ~((1 << WGM13)|(1 << WGM12)|(1 << CS12)|(1 << CS11)|(1 << CS10)); //reset timer settings
      TCCR1B |= (1 << WGM13)|(1 << CS11); //set up phase correct counter and counter for 1 prescaler
      ICR1H = 0;
      ICR1L = 255;        
      break;     
    default:
      break;
  }
    
  sei();//allow interrupts
}

static void enable_ISR(int sv_pin)
{
  switch(sv_pin) 
  {
    case 2: 
      TCCR3A &= ~((1 << COM3B1)|(1 << COM3B0)); //reset COM3B
      TCCR3A |= ((1 << COM3B1));
      OCR3B = 0;
      break;    
    case 3:
      TCCR3A &= ~((1 << COM3C1)|(1 << COM3C0)); //reset COM3C
      TCCR3A |= ((1 << COM3C1));
      OCR3B = 0;
      break;          
    case 5:
      TCCR3A &= ~((1 << COM3A1)|(1 << COM3A0)); //reset COM3A
      TCCR3A |= ((1 << COM3A1));
      OCR3A = 0;
      break;            
    case 6:
      TCCR4A &= ~((1 << COM4A1)|(1 << COM4A0)); //reset COM4A
      TCCR4A |= ((1 << COM4A1));
      OCR4A = 0;
      break;
    case 7:
      TCCR4A &= ~((1 << COM4B1)|(1 << COM4B0)); //reset COM4B
      TCCR4A |= ((1 << COM4B1));
      OCR4B = 0;
      break;
    case 8:
      TCCR4A &= ~((1 << COM4C1)|(1 << COM4C0)); //reset COM4C
      TCCR4A |= ((1 << COM4C1));
      OCR4C = 0;
      break;      
    case 11:
      TCCR1A &= ~((1 << COM1A1)|(1 << COM1A0)); //reset COM1A
      TCCR1A |= ((1 << COM1A1));
      OCR1A = 0;
      break;       
    case 12:
      TCCR1A &= ~((1 << COM1B1)|(1 << COM1B0)); //reset COM1B
      TCCR1A |= ((1 << COM1B1));
      OCR1B = 0;
      break;    
    case 13:
      TCCR1A &= ~((1 << COM1C1)|(1 << COM1C0)); //reset COM1C
      TCCR1A |= ((1 << COM1C1));
      OCR1C = 0;
      break;           
    default:
      break;
  }
}

static void disable_ISR(int sv_pin)
{
  switch(sv_pin) 
  {
    case 2: 
      TCCR3A &= ~((1 << COM3B0)|(1 << COM3B1));
      OCR3B = 0;    
      break;
    case 3:   
      TCCR3A &= ~((1 << COM3C0)|(1 << COM3C1));
      OCR3C = 0;
      break;      
    case 5:
      TCCR3A &= ~((1 << COM3A0)|(1 << COM3A1));
      OCR3A = 0;
      break;            
    case 6:
      TCCR4A &= ~((1 << COM4A0)|(1 << COM4A1));
      OCR4A = 0;
      break;
    case 7:
      TCCR4A &= ~((1 << COM4B0)|(1 << COM4B1));
      OCR4B = 0;
      break;
    case 8:
      TCCR4A &= ~((1 << COM4C0)|(1 << COM4C1));
      OCR4C = 0;
      break;   
    case 11:
      TCCR1A &= ~((1 << COM1A0)|(1 << COM1A1));
      OCR1A = 0;    
      break;           
    case 12:
      TCCR1A &= ~((1 << COM1B0)|(1 << COM1B1));
      OCR1B = 0;    
      break;    
    case 13:      
      TCCR1A &= ~((1 << COM1C0)|(1 << COM1C1));
      OCR1C = 0;   
      break;  
    default:
      break;
  }
}

BLS510::BLS510() 
{
   
}

void BLS510::attach(int sv_pin, int fr_pin, int en_pin, bool Orientation)
{
  this->sv_pin = sv_pin;
  this->fr_pin = fr_pin;
  this->en_pin = en_pin;

  this->Orientation = Orientation;

  pinMode( this->sv_pin, OUTPUT);
  pinMode( this->fr_pin, OUTPUT);
  pinMode( this->en_pin, OUTPUT);  

  init_ISR(this->sv_pin);
  enable_ISR(this->sv_pin);
  digitalWrite(en_pin, HIGH);
}

void BLS510::detach()
{
  disable_ISR(this->sv_pin);
  digitalWrite(en_pin, LOW);
}

bool BLS510::attached()
{
  bool ret = false;
  switch(this->sv_pin) 
  {
    case 6:
      if ((1 << COM4A0)|(1 << COM4A1)) 
      {
        ret = true;
      }
      break;
    case 7:
      if ((1 << COM4B0)|(1 << COM4B1))
      {
        ret = true;
      }
      break;
    case 8:
      if ((1 << COM4C0)|(1 << COM4C1)) 
      {
        ret = true;
      }
      break;      
    default:
      break;
  }
  return ret;
}

void BLS510::write(byte pwm, bool dir)
{
  digitalWrite(this->fr_pin, dir);
  switch(this->sv_pin) 
  {
    case 6:
      OCR4AH = 0;
      OCR4AL = pwm;
      break;
    case 7:
      OCR4BH = 0;
      OCR4BL = pwm;
      break;
    case 8:
      OCR4CH = 0;
      OCR4CL = pwm;
      break;      
    default:
      break;
  }
}

void BLS510::disable()
{
  digitalWrite(en_pin, LOW);
}

void BLS510::enable()
{
  digitalWrite(en_pin, HIGH);
}



  
