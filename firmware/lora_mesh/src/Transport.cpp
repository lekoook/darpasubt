#include <Transport.h>

namespace Transport
{
    /**
     * @brief Construct a new Transport:: Transport object.
     * 
     * @param src_addr address of source node.
     */
    Transport::Transport(uint8_t src_addr, RHMesh* net_manager) :
    source_addr(src_addr), 
    net_manager(net_manager)
    {
        this->chunk = 0;
        this->recv_chunk.
    }

    Transport::~Transport(void) {}

    /**
     * @brief Sends a data Chunk to a given destination address.
     * 
     * @param dest_addr address of destination node.
     * @param chunk Chunk containing the data to be sent.
     */
    void Transport::send(uint8_t dest_addr, Chunk::Chunk* chunk)
    {
        this->chunk = chunk;

        // Break data chunk into segments.
        this->chunk->segment_data(this->source_addr, dest_addr);
        // Send each segment one by one.
        for (uint8_t i = 0; i < this->chunk->get_seg_count(); i++)
        {
            uint8_t buf[SEGMENT_SIZE];
            uint8_t len = sizeof(buf);
            memset(buf, 0, SEGMENT_SIZE);

            this->chunk->flatten_seg(i, buf);
            net_manager->sendtoWait(buf, sizeof(buf), dest_addr);
            Serial.println(F("sent, waiting ACK"));

            if (net_manager->recvfromAckTimeout(buf, &len, ACK_TIMEOUT))
            {
                uint16_t ack = chunk->get_ack(buf);
                uint16_t seq = chunk->get_isn() + i;
                if (ack == seq)
                {
                    // ACK number is equal to the SEQ we just sent. Need to resend.
                    i--;
                    continue;
                }
                else if (ack > seq)
                {
                    // Send the Segment with the SEQ that is equal to the ACK.
                    i += ack - seq - 1;
                }
                else
                {
                    // ACK is less than the SEQ. We resend the Segment with SEQ equal to that ACK. 
                    i = seq - ack - 1;
                }
            }
            else
            {
                // We did not receive an ACK. Resend current Segment again.
                i--;
            }
            
            // for (int j = 0; j < SEGMENT_SIZE; j++)
            // {
            //     Serial.print(buf[j], HEX);
            // }
            // Serial.println(F(""));
        }
        // Serial.println(F(""));
    }

    /**
     * @brief Receives a data Chunk from a source address.
     * 
     * @param data bytes array chunk received.
     */
    void Transport::receive(uint8_t* data)
    {
        uint8_t source = 0;
        uint8_t buf[SEGMENT_SIZE];
        uint8_t len = sizeof(buf);
        memset(buf, 0, SEGMENT_SIZE);
        if (net_manager->recvfromAck(buf, &len, &source))
        {
            // TODO: Send back ACK.
            uint16_t seq_num = chunk->get_seq(buf);
            if (CRC::CRC::check_crc16(buf, SEGMENT_SIZE, CRC_IDX))
            {
                uint16_t ack_num = seq_num + 1;
                
            }
            
            Serial.print(F("from "));
            Serial.print(source);
            Serial.print(F(" : "));
            for (int j = 0; j < SEGMENT_SIZE; j++)
            {
                Serial.print((char)buf[j]);
            }
            Serial.println(F(""));
            if (CRC::CRC::check_crc16(buf, SEGMENT_SIZE, CRC_IDX))
                Serial.println(F("crc correct"));
            else
                Serial.println(F("crc wrong"));
        }
    }
}


