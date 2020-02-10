#include <SerialParser.h>

namespace SerialParser {

     PWMServo Servo0;
     PWMServo Servo1;

SerialParser::SerialParser() {
    this->currentLength = 0;
	this->parserState = ParserState::AWAITING_PACKET;
}

ParserState SerialParser::addByteAndCheckIfComplete(uint8_t byte){
	// Reset 
    if(this->currentLength > MAX_CHUNK_SIZE || parserState == ParserState::PACKET_INCOMPLETE || parserState == ParserState::PACKET_READY || parserState == ParserState::ERROR){
        this->currentLength = 0; 
		this->parserState = ParserState::AWAITING_PACKET;
        return ParserState::ERROR;
    }

	// first packet indentifier detection
    if(currentLength == 0 && this->parserState == ParserState::AWAITING_PACKET) {
        if(byte == (uint8_t) SerialResponseMessageType::PACKET_RECIEVED) {
			this->parserState = ParserState::PACKET_INCOMPLETE;
        	return ParserState::PACKET_INCOMPLETE; 
		} else if (byte == (uint8_t) SerialResponseMessageType::DROP_NODE) {
			parserState = ParserState::AWAITING_DROP_ID;
			currentLength++;
		} else {
            return ParserState::AWAITING_PACKET; 
		}
	// Drop node state
	} else if(parserState == ParserState::AWAITING_DROP_ID) {
		if(byte >=0 && byte <4) {
			drop_communication_node(byte);
			parserState = ParserState::PACKET_READY;
		} else {
			parserState = ParserState::ERROR;
			return ParserState::ERROR;
		}		
	// other
	} else {
		this->buffer[currentLength] = byte;
		currentLength++;
		if(currentLength == 3){
			desiredLength = buffer[1];
			desiredLength <<= 8;
			desiredLength += buffer[2];
			if(desiredLength > MAX_CHUNK_SIZE){
				return ParserState::ERROR;
			}
			return ParserState::PACKET_INCOMPLETE;
		}
		if(currentLength == desiredLength + 3 && currentLength > 3) {
			return ParserState::PACKET_READY;
		}
	}
	return ParserState::PACKET_INCOMPLETE;
}

Chunk::Chunk SerialParser::retrieveChunkAndReset(uint8_t myAddress) {
    Chunk::Chunk new_chunk(buffer+3,this->desiredLength, myAddress, buffer[0]);
    this->currentLength = 0;
    this->desiredLength = 0;
	this->parserState = ParserState::AWAITING_PACKET;
    return new_chunk;
}

SerialResponsePacket::SerialResponsePacket(Chunk::Chunk chunk) {
    this->length = chunk.get_len() + 5;
    buffer[0] = (uint8_t)SerialResponseMessageType::PACKET_RECIEVED;
    buffer[1] = (uint8_t)chunk.get_src(); //RESERVE ONLY 1 BYTE FOR ADDRESS
    buffer[2] = chunk.get_len() >> 8;
    buffer[3] = chunk.get_len() & 0xFF;
    memcpy(buffer+4, chunk.get_data(), chunk.get_len());
    buffer[chunk.get_len() + 4] = chunk.get_rssi();
}

SerialResponsePacket::SerialResponsePacket(uint16_t concentration, float humidity, float temperature) {
	this->length = 12;
	buffer[0] = (uint8_t) SerialResponseMessageType::CO2_SENSOR_READING;
	buffer[1] = 0xff; // alignment
	memcpy(buffer+2, &concentration, sizeof(uint16_t));
	memcpy(buffer+4, &humidity, sizeof(float));
	memcpy(buffer+8, &temperature, sizeof(float));
}

uint8_t* SerialResponsePacket::serialize(){
    return buffer;
}

int SerialResponsePacket::getLength() {
    return this->length;
}
void drop_communication_node(uint8_t device) {
      switch(device) {
          case 0:
              Servo0.write(30);
              break;
          case 1:
              Servo1.write(30);
              break;
      }
}

void setup_servos() {
	// Servo setup
    Servo0.attach(SERVO0_PIN);
    Servo1.attach(SERVO1_PIN);
	Servo0.write(0);
	Servo1.write(0);

}

}
