#ifndef config_h
#define config_h

// Used to tune movement for different motors and connections
// Sometimes directions are inverted
// Motor Speeds
#define LEFT_MOTOR_SPEED_SCALING (1.55)//(3.3)
#define RIGHT_MOTOR_SPEED_SCALING (1.55)//(5.0)
#define MOTOR_GEAR_REDUCTION (40.0)

#define LEFT_ENCODER_SCALING (-53.0)
#define RIGHT_ENCODER_SCALING (53.0)

#define MAX_RPM 3000 //2700
#define WHEEL_DIAMETER 0.180 //0.180
#define FR_WHEELS_DISTANCE 0.0 //0.264
#define LR_WHEELS_DISTANCE 0.534 //0.534


#endif
