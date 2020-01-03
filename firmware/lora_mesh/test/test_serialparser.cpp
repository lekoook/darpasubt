#include <Arduino.h>
#include <unity.h>
#include <SerialParser.h>

void test_serialParserHappyCase() {
    SerialParser::SerialParser parser;
    SerialParser::ParserState res;
    uint8_t testPacket[4] = {0, 0, 1, 6};
    for(int i = 0; i < 4; i++){
        res = parser.addByteAndCheckIfComplete(testPacket[i]);
        if(i < 3) {
            TEST_ASSERT_EQUAL(SerialParser::ParserState::PACKET_INCOMPLETE, res);
        }
    }

    TEST_ASSERT_EQUAL(SerialParser::ParserState::PACKET_READY, res);
}

void setup(){
    delay(2000);
    UNITY_BEGIN();    // IMPORTANT LINE!
    RUN_TEST(test_serialParserHappyCase);
}

void loop(){

}