// Mesh network implementation using the Grove LoRa RFM95 module.

#include <RH_RF95.h>
#include <RHMesh.h>
#include <common.h>
#include <Transport.h>
#include <main.h>
#include <LoraPacket.h>
#include <FreeRTOS_TEENSY4.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <SoftwareSerial.h>
#include <SerialParser.h>

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


#ifdef USE_SOFTWARE_SERIAL
/**
 * @brief Configures network with defined values. For Software Serial.
 * 
 * @param driver radio driver used.
 * @param mesh_manager mesh network manager used.
 */
void config_network(RH_RF95<SoftwareSerial>* driver, RHMesh* mesh_manager)
{
    driver->setFrequency(434.0);
    driver->setTxPower(TX_POWER);
    mesh_manager->setRetries(RE_TX_RETRIES);
    mesh_manager->setTimeout(RE_TX_TIMEOUT);
    mesh_manager->setArpTimeout(MESH_ARP_TIMEOUT);
}

SoftwareSerial soft_serial (RX_SOFTSERIAL_PIN, TX_SOFTSERIAL_PIN);
RH_RF95<SoftwareSerial> driver(soft_serial);
#endif



///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////



#ifdef USE_HARDWARE_SERIAL
/**
 * @brief Configures network with defined values. For Hardware Serial.
 * 
 * @param driver radio driver used.
 * @param mesh_manager mesh network manager used.
 */
void config_network(RH_RF95<HardwareSerial>* driver, RHMesh* mesh_manager)
{
    driver->setFrequency(434.0);
    driver->setTxPower(TX_POWER);
    mesh_manager->setRetries(RE_TX_RETRIES);
    mesh_manager->setTimeout(RE_TX_TIMEOUT);
    mesh_manager->setArpTimeout(MESH_ARP_TIMEOUT);
}

RH_RF95<HardwareSerial> driver(HARD_SERIAL);
#endif



///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////



/* Local function prototypes */
void data_recv_cb(const talker_pkg::LoraPacket& to_transmit);
void pub_data(Chunk::Chunk& chunk);
int32_t str_to_int(const char* str);
uint32_t int_to_str(char* str, int32_t integer);
void serial_spin();

// Class to manage message delivery and receipt, using the driver declared above
RHMesh mesh_manager(driver, MESH_ADDRESS);
Transport::Transport transporter(MESH_ADDRESS, &mesh_manager);
Chunk::Chunk recv_chunk;

// ROS related
/*ros::NodeHandle nh;
talker_pkg::LoraPacket pub_msg;
ros::Subscriber<talker_pkg::LoraPacket> sub("tx", &data_recv_cb);
ros::Publisher pub("rx", &pub_msg);*/

// CO2 sensor
SCD30 airSensor;

// measurement interval in seconds >= 1
#define AIR_MEAS_INTERVAL 2 

void setup() 
{
    SHOW_SERIAL.begin(SERIAL_BAUD_RATE);

    while (!mesh_manager.init()) {}
    config_network(&driver, &mesh_manager);

    #ifdef DEBUG_PRINT
    SHOW_SERIAL.println(F(MSG_INIT_SUCCESS));
    SHOW_SERIAL.println(F(MSG_WHOAMI));
    #endif

    pinMode(LED_BUILTIN, OUTPUT);

	// CO2 sensor setup
	Wire.begin();
	airSensor.begin();
	airSensor.setMeasurementInterval(AIR_MEAS_INTERVAL);
	
	// drop node setup
	SerialParser::setup_servos();
}

void loop()
{
    
    serial_spin();
    
    /*transporter.process_send_queue();
    if (transporter.receive(driver))
    {
        if (transporter.get_one_chunk(recv_chunk))
        {
            pub_data(recv_chunk);
        }
    }*/

	// CO2, humidity and temperature measurements
	if(airSensor.dataAvailable()) {
	 // Copy over the payload.
		SerialParser::SerialResponsePacket  packet(airSensor.getCO2(), airSensor.getHumidity(), airSensor.getTemperature());
		uint8_t* serialPacket = packet.serialize();
		int length = packet.getLength();
		if(Serial.dtr()) {
			Serial.write(serialPacket, length);
		}
	} else {
		Serial.println("no data from sensor");
	}
		
	//delay(1000);


}

/**
 * Handle serial data
 */ 
void serial_spin(){

    static SerialParser::SerialParser parser;

    if(Serial.available()) {
        uint8_t byte = Serial.read();
        SerialParser::ParserState res = parser.addByteAndCheckIfComplete(byte); 
        if(res == SerialParser::ParserState::PACKET_READY){
            //transporter.queue_chunk();
            Chunk::Chunk chunk = parser.retrieveChunkAndReset(MESH_ADDRESS);
            chunk.set_rssi(100);
            chunk.set_src(1);
            pub_data(chunk);
        }

    }
}
/**
 * @brief Publishes Chunk payload data to our custom protocol bus
 * 
 * @param chunk reference to Chunk containing the data.
 */
void pub_data(Chunk::Chunk& chunk)
{
    // Copy over the payload.
    SerialParser::SerialResponsePacket  packet(chunk);
    uint8_t* serialPacket = packet.serialize();
    int length = packet.getLength();
    if(Serial.dtr()) {
        Serial.write(serialPacket, length);
    }
}

/**
 * @brief Converts a C-string into an int. Returns -1 if string is not an int.
 * TODO: Account for negative numbers.
 * 
 * @param str C-string to convert. 
 * @return int32_t converted int.
 */
int32_t str_to_int(const char* str)
{
    uint16_t num = 0;
    bool valid = false;
    while (*str >= '0' && *str <= '9')
    {
        valid = true;
        num *= 10;
        num += *str - '0';
        str++;
    }
    if (!valid) return -1;
    return num;
}

/**
 * @brief Converts an int into a C-string, with no guarantee of null termination.
 * 
 * @param str pointer to char array to store converted string.
 * @param integer int to convert.
 * @return uint32_t the length of converted string.
 */
uint32_t int_to_str(char* str, int32_t integer)
{
    return (uint32_t) sprintf(str, "%ld", integer);
}



