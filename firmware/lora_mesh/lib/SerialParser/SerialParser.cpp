#include <SerialParser.h>

namespace SerialParser {

SerialParser::SerialParser() {
    this->currentLength = 0;
    this->awaitingPackets = true;
}

ParserState SerialParser::addByteAndCheckIfComplete(uint8_t byte){
    if(this->currentLength > MAX_CHUNK_SIZE){
        this->currentLength = 0; 
        this->awaitingPackets = true;
        return ParserState::ERROR;
    }
    if(this->currentLength == 0 && this->awaitingPackets) {
        if(byte != 0xFA)
            return ParserState::AWAITING_PACKET; 
        awaitingPackets = false;
        return ParserState::PACKET_INCOMPLETE;
    }
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
    return ParserState::PACKET_INCOMPLETE;
}

Chunk::Chunk SerialParser::retrieveChunkAndReset(uint8_t myAddress) {
    Chunk::Chunk new_chunk(buffer+3,this->desiredLength, myAddress, buffer[0]);
    this->currentLength = 0;
    this->desiredLength = 0;
    this->awaitingPackets = true;
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

SerialResponsePacket::SerialResponsePacket(LoraStatusReady ready){
    this->length = 1;
    buffer[0] = (uint8_t)SerialResponseMessageType::LORA_STATUS_READY;
}

SerialResponsePacket::SerialResponsePacket(MeshAddress mesh) {
    this->length = 2;
    buffer[0] = (uint8_t)SerialResponseMessageType::PHYSICAL_ADDRESS;
    buffer[1] = mesh.address;
}

SerialResponsePacket::SerialResponsePacket(SonarReading reading) {
    this->length = 3;
    if(reading.sonarLocation == SonarLocation::SONAR_BOTTOM)
        buffer[0] = (uint8_t)SerialResponseMessageType::SONAR_BOTTOM; 
    else if(reading.sonarLocation == SonarLocation::SONAR_FRONT)
        buffer[0] = (uint8_t)SerialResponseMessageType::SONAR_FRONT;
    buffer[1] = reading.range >> 8;
    buffer[2] = reading.range  & 0XFF;
}
uint8_t* SerialResponsePacket::serialize(){
    return buffer;
}

int SerialResponsePacket::getLength() {
    return this->length;
}

}