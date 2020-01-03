#ifndef _SERIAL_PARSER_H_
#define _SERIAL_PARSER_H_

#include <stdint.h>
#include <Chunk.h>

#define MAX_CHUNK_SIZE 240

namespace SerialParser{
    enum class ParserState {
        PACKET_INCOMPLETE,
        PACKET_READY,
        ERROR
    };
    class SerialParser {
        uint8_t buffer[MAX_CHUNK_SIZE];
        uint16_t currentLength;
        uint16_t desiredLength;
    public:
        SerialParser();
        /**
         * Add a byte as read from the Serial Input to the parser
         * @returns bool True if packet is complete and ready to 
         * be sent as a chunk
         */ 
        ParserState addByteAndCheckIfComplete(uint8_t byte);
        /**
         * Retrieve the latest chunk and reset the Parser
         */ 
        Chunk::Chunk retrieveChunkAndReset(uint8_t myAddress);
    };
}

#endif