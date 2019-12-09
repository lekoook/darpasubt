#include <Arduino.h>
#include <crc.h>

#define HEADER_SIZE 10
#define PAYLOAD_SIZE 40
#define SEGMENT_SIZE (HEADER_SIZE) + (PAYLOAD_SIZE)
#define SRC_IDX 0
#define DEST_IDX 1
#define OFFSET_IDX 2
#define SEQ_IDX 4
#define ACK_IDX 6
#define CRC_IDX 8
#define PAYL_IDX 10

// NOTE: Tested 150, its stretching SRAM limit.
#define MAX_DATA_SIZE 100
#define MAX_SEG_CNT (((MAX_DATA_SIZE) / (PAYLOAD_SIZE)) + 1)

/**
 * Segment structure
 * 
 * flags:
 *      |*****|*****|*****|*****|*****|*****|*****|*****|
 *      | ACK | ACK | ACK | ACK | ACK | ACK | ACK | ACK |
 *      |*****|*****|*****|*****|*****|*****|*****|*****|
 */

namespace Chunk
{
    struct Segment 
    {
        uint16_t seq;
        uint16_t offset;
        uint8_t payload[PAYLOAD_SIZE];
        Segment(void);
        ~Segment(void);
    };

    class Chunk
    {
        private:
            uint16_t id;
            uint8_t data[MAX_DATA_SIZE];
            uint16_t len;
            Segment segments[MAX_SEG_CNT];
            uint8_t segments_count;
            uint16_t isn;
            
            uint8_t source;
            uint8_t dest;
            uint16_t ack;//
            uint8_t flags;

        public:
            Chunk(void);
            Chunk(uint8_t* data, uint16_t len);
            ~Chunk(void);
            uint16_t get_seg_count(void);
            void segment_data(uint8_t& src_addr, uint8_t& dest_addr);
            void flatten_seg(uint8_t index, uint8_t buf[SEGMENT_SIZE]);
            uint16_t gen_isn(void);
    };
}