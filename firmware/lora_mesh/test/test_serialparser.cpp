#include <Arduino.h>
#include <unity.h>
#include <SerialParser.h>

void test_serialParserHappyCase() {
    SerialParser::SerialParser parser;
    SerialParser::ParserState res;
    uint8_t testPacket[5] = {0xFA ,0, 0, 1, 6};
    for(int i = 0; i < 5; i++){
        res = parser.addByteAndCheckIfComplete(testPacket[i]);
        if(i < 4) {
            TEST_ASSERT_EQUAL(SerialParser::ParserState::PACKET_INCOMPLETE, res);
        }
    }

    TEST_ASSERT_EQUAL(SerialParser::ParserState::PACKET_READY, res);

    Chunk::Chunk chunk = parser.retrieveChunkAndReset(1);
    uint8_t expected_buffer[6] = {0xFA, 0x1, 0x0, 0x1, 0x6, 0x0};
    expected_buffer[5] = chunk.get_rssi();
    SerialParser::SerialResponsePacket packet(chunk);
    int length = packet.getLength();
    TEST_ASSERT_EQUAL(6, length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_buffer, packet.serialize(), 6);
}

void test_serialParserAbove64Bytes() {
    SerialParser::SerialParser parser;
    SerialParser::ParserState res;
    const int LENGTH = 100;
    uint8_t testPacketHeader[4] = {0xFA, 0, 0, LENGTH};
    for(int i = 0; i < sizeof(testPacketHeader); i++){
        res = parser.addByteAndCheckIfComplete(testPacketHeader[i]);
        TEST_ASSERT_EQUAL(SerialParser::ParserState::PACKET_INCOMPLETE, res);
    }
    for(uint8_t i = sizeof(testPacketHeader); i < LENGTH + sizeof(testPacketHeader); i++){
        res = parser.addByteAndCheckIfComplete(i);
    }
    TEST_ASSERT_EQUAL(SerialParser::ParserState::PACKET_READY, res);

    Chunk::Chunk chunk = parser.retrieveChunkAndReset(0);
    uint8_t expected_buffer[200];
    for(int i  = 0 ; i < sizeof(expected_buffer); i++ ){
        expected_buffer[i] = 0;
    }
    for(uint8_t i = sizeof(testPacketHeader); i < LENGTH + sizeof(testPacketHeader); i++){
        expected_buffer[i] = i;
    }
    memcpy(expected_buffer, testPacketHeader, sizeof(testPacketHeader));
    expected_buffer[sizeof(testPacketHeader)+chunk.get_len()] = chunk.get_rssi();
    SerialParser::SerialResponsePacket packet(chunk);
    int length = packet.getLength();
    TEST_ASSERT_EQUAL(LENGTH+sizeof(testPacketHeader)+1, length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_buffer, packet.serialize(), LENGTH+sizeof(testPacketHeader));
    TEST_ASSERT_EQUAL(LENGTH + sizeof(testPacketHeader) - 1, packet.serialize()[packet.getLength()-2]);
}

void setup(){
    delay(2000);
    UNITY_BEGIN();    // IMPORTANT LINE!
    RUN_TEST(test_serialParserHappyCase);
    RUN_TEST(test_serialParserAbove64Bytes);
    UNITY_END();
}

void loop(){

}