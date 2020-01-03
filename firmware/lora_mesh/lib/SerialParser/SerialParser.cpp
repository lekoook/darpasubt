#include <SerialParser.h>

namespace SerialParser {

SerialParser::SerialParser() {
    this->currentLength = 0;
}

ParserState SerialParser::addByteAndCheckIfComplete(uint8_t byte){
    if(this->currentLength > MAX_CHUNK_SIZE){
        this->currentLength = 0; 
        return ParserState::ERROR;
    }
    this->buffer[currentLength] = byte;
    currentLength++;
    if(currentLength == 3){
        desiredLength = buffer[1];
        desiredLength <<= 8;
        desiredLength += buffer[2];
        return ParserState::PACKET_INCOMPLETE;
    }
    if(currentLength == desiredLength + 3 && currentLength > 3) {
        return ParserState::PACKET_READY;
    }
    return ParserState::PACKET_INCOMPLETE;
}

Chunk::Chunk SerialParser::retrieveChunkAndReset(uint8_t myAddress) {
    Chunk::Chunk new_chunk(buffer+3,this->desiredLength, myAddress, buffer[0]);
    return new_chunk;
}
}