#include "ThermalImager.h"

bool isConnected(uint8_t addr) {
    Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

void setupThermal(uint8_t addr) {
    Wire.begin();
    Wire.setClock(400000);
    if (!isConnected(addr)) {
        Serial.print("Thermal camera not connected at addr ");
        Serial.println(addr);
    } else {
        Serial.print("Thermal camera connected at ");
        Serial.println(addr);
    }
}

void getEeparams(uint8_t addr, paramsMLX90640 *params) {
    uint16_t eedata[832];
    MLX90640_DumpEE(addr, eedata);
    MLX90640_ExtractParameters(eedata, params);
}

void flip(float result[]) {
    for (int i = 0; i < 24; i++) {
        int start = i*32;
        int end = i*32 + 31;
        float temp;
        while(start < end) {
            temp = result[start];
            result[start] = result[end];
            result[end] = temp;
            start++;
            end--;
        }
    }
}

void flipVert (float result[]) {
    for (int i = 0; i < 32; i++) {
        int start = i;
        int end = i + (23 * 32);
        float temp;
        while (start < end) {
            temp = result[start];
            result[start] = result[end];
            result[end] = result[start];
            start += 32;
            end -= 32;
        }
    }
}

void getFrameData(uint8_t addr, paramsMLX90640 *params, float result[]) {
    for (byte x = 0; x < 2; x++) {
        uint16_t frameData[834];
        int status = MLX90640_GetFrameData(addr, frameData);
        float Ta = MLX90640_GetTa(frameData, params);
        float tr = Ta - 8;
        float emissivity = 0.95;
        MLX90640_CalculateTo(frameData, params, emissivity, tr, result);
    }
    flipVert(result);
    // flip(result);
}

void floatToUint8(float res[], uint8_t asciiRes[]) {
    for (int i = 0; i < 768; i++) {
        if (res[i] > 0) {
            asciiRes[i] = (uint8_t)round(res[i]);
        } else {
            asciiRes[i] = 0;
        }
    }
}

void printResult(float res[]) {
    for (int i = 0; i < 24; i++) {
        Serial.println();
        for (int j = 0; j < 32; j++) {
            Serial.print(res[i*32 + j]);
            Serial.print("|");
        }
    }
}

void printResult(uint8_t res[]) {
    for (int i = 0; i < 24; i++) {
        Serial.println();
        for (int j = 0; j < 32; j++) {
            Serial.print(res[i*32 + j]);
            Serial.print("|");
        }
    }
}

void printSerial(uint8_t res[]) {
    Serial.println(255);
    for (int i = 0; i < 24; i++) {
        Serial.println();
        for (int j = 0; j < 32; j++) {
            Serial.print(res[i*32 + j]);
        }
    }
}

void pubThermal(uint8_t res[]) {
    uint8_t slice[192];
    for (int i = 0; i < 4; i++) {
        memcpy(slice, res + (i * 192), 192);
        SerialParser::SerialResponsePacket packet(slice, i, 192);
        uint8_t* serialPacket = packet.serialize();
        int length = packet.getLength();
        if(Serial.dtr()) {
            Serial.write(serialPacket, 195);
        }
    }
}