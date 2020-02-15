#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "SerialParser.h"
#include <Wire.h>

void setupThermal(uint8_t addr);
void getEeparams(uint8_t addr, paramsMLX90640 *params);
void getFrameData(uint8_t addr, paramsMLX90640 *params, float result[]);
void floatToUint8(float res[], uint8_t asciiRes[]);
void printResult(float res[]);
void printResult(uint8_t res[]);
void printSerial(uint8_t res[]);
void pubThermal(uint8_t res[]);