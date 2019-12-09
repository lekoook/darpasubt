#include <Chunk.h>

namespace Chunk
{
    /**
     * @brief Construct a new Segment:: Segment struct.
     * 
     */
    Segment::Segment(void) : seq(0), offset(0)
    {
        for (int i = 0; i < PAYLOAD_SIZE; i++)
        {
            payload[i] = 0;
        }
    }

    Segment::~Segment(void) {}

    /**
     * @brief Construct a new Chunk:: Chunk object.
     * 
     * @param data bytes array to store as a Chunk.
     * @param len length of bytes array (in bytes).
     */
    Chunk::Chunk(uint8_t* data, uint16_t len) : len(len)
    {
        this->segments_count = 0;
        memset(this->data, 0, MAX_DATA_SIZE);
        memset(this->segments, 0, sizeof(Segment) * MAX_SEG_CNT);
        for (uint16_t i = 0; i < len; i++)
        {
            this->data[i] = data[i];
        }
        this->id = CRC::CRC::calc_crc16(data, len);
        this->isn = gen_isn();
    }

    Chunk::Chunk(void) {}
    Chunk::~Chunk(void) {}

    /**
     * @brief Returns the number of Segments in this Chunk.
     * 
     * @return uint16_t number of Segments.
     */
    uint16_t Chunk::get_seg_count(void)
    {
        return this->segments_count;
    }

    /**
     * @brief Breaks a Chunk into Segments.
     * 
     * @param src_addr address of source node.
     * @param dest_addr address of destination node.
     */
    void Chunk::segment_data(uint8_t& src_addr, uint8_t& dest_addr)
    {
        this->source = src_addr;
        this->dest = dest_addr;

        this->ack = 0; // Change
        
        this->segments_count = (len + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;
        
        for (uint8_t i = 0; i < this->segments_count; i++)
        {
            Segment seg;

            uint16_t pl_len = PAYLOAD_SIZE;
            uint16_t byte_idx = i * PAYLOAD_SIZE;

            seg.seq = isn + i;
            seg.offset = byte_idx / 8;
            if (len - byte_idx < PAYLOAD_SIZE) pl_len = len - byte_idx;
            memcpy(&(seg.payload), &data[i * PAYLOAD_SIZE], pl_len);

            this->segments[i] = seg;
        }
    }

    /**
     * @brief Flattens a Segment into an array.
     * 
     * @param index index position of the segment to flatten.
     * @param buf array to hold flattened Segment.
     */
    void Chunk::flatten_seg(uint8_t index, uint8_t buf[SEGMENT_SIZE])
    {
        // For last Segment.
        if (index == this->segments_count-1)
            this->flags &= 0xFE; // More Segment bit is cleared to '0'.
        else
            this->flags |= 0x01; // More Segment bit is set to '1'.

        uint16_t seg_status = segments[index].offset; // Segmentation offset.
        seg_status |= (this->flags << 5); // Segmentation flags.
        
        // Copy all Segment data into transmission buffer.
        buf[SRC_IDX] = this->source;
        buf[DEST_IDX] = this->dest;
        memcpy(&buf[OFFSET_IDX], &seg_status, sizeof(uint16_t));
        memcpy(&buf[SEQ_IDX], &(this->segments[index].seq), sizeof(uint16_t));
        memcpy(&buf[ACK_IDX], &(this->ack), sizeof(uint16_t));
        memcpy(&buf[PAYL_IDX], &(this->segments[index].payload), PAYLOAD_SIZE);

        CRC::CRC::append_crc16(buf, SEGMENT_SIZE, CRC_IDX);
    }

    /**
     * @brief Generates a random initial sequence number.
     * 
     * @return uint16_t random initial sequence.
     */
    uint16_t Chunk::gen_isn(void)
    {
        randomSeed(this->id);
        return (uint16_t)random(0, sizeof(uint16_t));
    }
}