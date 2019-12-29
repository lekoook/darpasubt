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
void pub_data(Chunk::Chunk& chunk);
int32_t str_to_int(const char* str);
uint32_t int_to_str(char* str, int32_t integer);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh mesh_manager(driver, MESH_ADDRESS);
Transport::Transport transporter(MESH_ADDRESS, &mesh_manager);
Chunk::Chunk recv_chunk;

// ROS related
ros::NodeHandle nh;
talker_pkg::LoraPacket pub_msg;
ros::Subscriber<talker_pkg::LoraPacket> sub("tx", &data_recv_cb);
ros::Publisher pub("rx", &pub_msg);

void setup() 
{
    SHOW_SERIAL.begin(SERIAL_BAUD_RATE);

    while (!mesh_manager.init()) {}
    config_network(&driver, &mesh_manager);

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);

    #ifdef DEBUG_PRINT
    SHOW_SERIAL.println(F(MSG_INIT_SUCCESS));
    SHOW_SERIAL.println(F(MSG_WHOAMI));
    #endif

    pinMode(LED_BUILTIN, OUTPUT);
}


void loop()
{
    transporter.process_send_queue();

    if (transporter.receive(driver))
    {
        if (transporter.get_one_chunk(recv_chunk))
        {
            pub_data(recv_chunk);
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
    int32_t dest = str_to_int(to_transmit.to.data);
    // Fail silently if destination address is negative.
    if (dest >= 0)
    {
        Chunk::Chunk new_chunk(to_transmit.data, to_transmit.data_length, dest);
        transporter.queue_chunk(new_chunk);
    }
}

/**
 * @brief Publishes Chunk payload data onto ROS.
 * 
 * @param chunk reference to Chunk containing the data.
 */
void pub_data(Chunk::Chunk& chunk)
{
    // Copy over the payload.
    memcpy(pub_msg.data, chunk.get_data(), chunk.get_len());
    pub_msg.data_length = chunk.get_len();
    
    // Point to the source address of payload.
    char src[12] = {0}; // Enough to store a 32 bit int.
    uint32_t len = int_to_str(src, chunk.get_src());
    src[len] = '\0'; // Ensure string is null terminated.
    pub_msg.from.data = src;

    // Point to the destination (this device) address of payload.
    char dest[12] = {0};
    len = int_to_str(dest, chunk.get_dest());
    dest[len] = '\0'; // Ensure string is null terminated.
    pub_msg.to.data = dest;

    // Copy over the RSSI value.
    pub_msg.rssi = chunk.get_rssi();
    pub.publish(&pub_msg);
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