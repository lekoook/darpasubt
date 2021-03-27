#include "UB510Motor.hpp"

/**
 * @brief Initializes the registers for PWM output.
 * 
 * @param pinSv Pin to initialize PWM for.
 */
static void initPWM(int pinSv)
{
    cli();
    switch(pinSv) 
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
    sei();
}

/**
 * @brief Enables the pin for PWM output.
 * 
 * @param pinSv Pin to enable PWM for.
 */
static void enablePWM(int pinSv)
{
    switch(pinSv) 
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

/**
 * @brief Disables the pin PWM output.
 * 
 * @param pinSv Pin to disable PWM for.
 */
static void disablePWM(int pinSv)
{
    switch(pinSv) 
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

void UB510Motor::attach(int pinSv, int pinFr, int pinEn, bool orientation)
{
        this->pinSv = pinSv;
        this->pinFr = pinFr;
        this->pinEn = pinEn;
        this->orientation = orientation;
        pinMode(pinSv, OUTPUT);
        pinMode(pinFr, OUTPUT);
        pinMode(pinFr, OUTPUT);
        digitalWrite(pinEn, HIGH);
        initPWM(pinSv);
        enablePWM(pinSv);
}

void UB510Motor::detach()
{
    digitalWrite(pinEn, LOW);
    disablePWM(pinSv);
}

void UB510Motor::set(unsigned char pwm, bool direction)
{
    set(direction);
    set(pwm);
}

void UB510Motor::set(unsigned char pwm)
{
    switch (pinSv)
    {
    case 2:
        OCR3BH = 0;
        OCR3BL = pwm;
        break;
    case 3:
        OCR3CH = 0;
        OCR3CL = pwm;
        break;
    case 5:
        OCR3AH = 0;
        OCR3AL = pwm;
        break;
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
    case 11:
        OCR1AH = 0;
        OCR1AL = pwm;
        break;
    case 12:
        OCR1BH = 0;
        OCR1BL = pwm;
        break;
    case 13:
        OCR1CH = 0;
        OCR1CL = pwm;
        break;
    default:
        break;
    }
}

void UB510Motor::set(bool direction)
{
    this->direction = orientation ? direction : !direction;
    digitalWrite(pinFr, this->direction);
}

bool UB510Motor::getDirection()
{
    return this->direction;
}

void UB510Motor::enable()
{
    digitalWrite(pinEn, HIGH);
}

void UB510Motor::disable()
{
    digitalWrite(pinEn, LOW);
}
