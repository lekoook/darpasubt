#ifndef _SERIAL_PARSER_H_
#define _SERIAL_PARSER_H_

#include <stdint.h>
#include <Chunk.h>
#include <PWMServo.h>

#define MAX_CHUNK_SIZE 240

namespace SerialParser {

#define SERVO0_PIN 2
#define SERVO1_PIN 3
	extern PWMServo Servo0;
	extern PWMServo Servo1;

	void drop_communication_node(uint8_t device);
	void setup_servos();

    enum class ParserState {
        PACKET_INCOMPLETE,
        PACKET_READY,
        ERROR,
        AWAITING_PACKET, // actually waiting for first packet only
		AWAITING_DROP_ID
	};
    enum class SerialResponseMessageType: uint8_t {
        PACKET_RECIEVED = 0xFA,
        PACKET_SENT = 0xFB,
        LORA_STATUS_READY = 0xFC,
        LORA_STATUS_BUSY = 0xFD,
        CO2_SENSOR_READING = 0xFE,
        THERMAL_FRONT = 0xFF,
        THERMAL_TOP = 0xF1,
        DEBUG = 0xF2,
		DROP_NODE = 0xF3
    };

    /**
     * Serial Parser protocol: 
     */ 
    class SerialParser {
		SerialResponseMessageType messageType;
		ParserState parserState;
        uint16_t desiredLength;
        uint16_t currentLength;
        uint8_t buffer[MAX_CHUNK_SIZE];
    public:
   
        SerialParser();

        /**
         * Add a byte as read from the Serial Input to the parser
         * Packet format expected is
         * [ START    |   TO    |   LENGTH     |      DATA        ]
         * [ 1 byte   | 1 byte  |   2 bytes    |     n bytes      ]
         * @returns ParserState of packet i.e if it is complete and ready to 
         * be sent as a chunk
         */ 
        ParserState addByteAndCheckIfComplete(uint8_t byte);
        /**
         * Retrieve the latest chunk and reset the Parser
         */
        Chunk::Chunk retrieveChunkAndReset(uint8_t myAddress);
    };

    /**
     * SerialResponsePacket is the representation for relaying Data from the
     * 
     * [0xFA       |  FROM    |   LENGTH     |      DATA        |  RSSI   ]
     * [1 byte     |  1 byte  |   2 bytes    |     n bytes      |  1 byte ]
     * 
     * [0xF1 | ]
	 *
     * SerialResponsePacket for CO2 SENSOR READING
     * 
     * [0xFE       |  0xff    |   Concentration |    HUMIDITY    |  TEMPERATURE ]
     * [1 byte     |  1 byte  |     uint16_t    |     float      |     float    ]
     * 
     * [0xF1 | ]
    */ 
    class SerialResponsePacket {
        uint8_t buffer[256];
        int length;
    public:
        SerialResponsePacket(Chunk::Chunk chunk);
        SerialResponsePacket(uint16_t concentration, float humidity, float temperature);
        uint8_t* serialize();
        int getLength();
    };
   }

#endif
