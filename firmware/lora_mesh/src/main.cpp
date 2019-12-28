// Mesh network implementation using the Grove LoRa RFM95 module.

#include <RH_RF95.h>
#include <RHMesh.h>
#include <SoftwareSerial.h>
#include <common.h>
#include <Transport.h>
#include <main.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <LoraPacket.h>

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
int32_t str_to_uint(const char* str);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh mesh_manager(driver, MESH_ADDRESS);
Transport::Transport transporter(MESH_ADDRESS, &mesh_manager);
Chunk::Chunk recv_chunk;

// ROS related
ros::NodeHandle nh;
ros::Subscriber<talker_pkg::LoraPacket> sub("tx", &data_recv_cb);

void setup() 
{
    SHOW_SERIAL.begin(SERIAL_BAUD_RATE);

    while (!mesh_manager.init()) {}
    config_network(&driver, &mesh_manager);

    nh.initNode();
    nh.subscribe(sub);

    #ifdef DEBUG_PRINT
    SHOW_SERIAL.println(F(MSG_INIT_SUCCESS));
    SHOW_SERIAL.println(F(MSG_WHOAMI));
    #endif

    pinMode(LED_BUILTIN, OUTPUT);
}


void loop()
{
    transporter.process_send_queue();

    if (transporter.receive())
    {
        if (transporter.get_one_chunk(recv_chunk))
        {
            for (int i = 0; i < recv_chunk.get_len(); i++)
            {
                SHOW_SERIAL.println(*(recv_chunk.get_data() + i), HEX);
            }
        }
    }

    nh.spinOnce();
}

/**
 * @brief Callback function for receiving data to be transmitted from rosserial.
 * 
 * @param to_transmit reference to the received data.
 */
void data_recv_cb(const talker_pkg::LoraPacket& to_transmit)
{
    // Create a new chunk and push to queue.
    int32_t dest = str_to_uint(to_transmit.to.data);
    // Fail silently if destination address is negative.
    if (dest >= 0)
    {
        Chunk::Chunk new_chunk(to_transmit.data, to_transmit.data_length, dest);
        transporter.queue_chunk(new_chunk);
    }
}

/**
 * @brief Converts a std_msgs::String object into an int. Returns -1 if string is not an int.
 * TODO: Account for negative numbers.
 * 
 * @param str C-string to convert. 
 * @return int32_t converted int.
 */
int32_t str_to_uint(const char* str)
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