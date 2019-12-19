#ifndef _HEADER_TEMPLATE_H
#define _HEADER_TEMPLATE_H

#include "deca_types.h"
#include "string.h"

enum
{
  HEADER_LEN = 10,
  CRC_LEN = 2,
  IDX_SEQ = 2,
  IDX_DEST = 5,
  IDX_SRC = 6,
  IDX_TYPE = 7,
  IDX_PAYLOAD = 10
};

enum message_type_t
{
  DISC_TYPE = 0x0,
  SYNC_TYPE = 0x1,
  RNG_TYPE = 0x2
};

struct header_template_t
{
  uint16 frame_ctrl;
  uint8 seq;
  uint16 pan_id;
  uint8 dest;
  uint8 src;
  uint8 type;
  uint16 reserved;
};

/**
 * Header anatomy:
 * 
 * |*******************|*************|************|**********|*********|**********|**********|
 * | Frame Control (2) | Seq No. (1) | PAN ID (2) | Dest (1) | Src (1) | Type (1) | Rsrv (2) |
 * |*******************|*************|************|**********|*********|**********|**********|
 * 
 * Frame control (2 bytes)
 * 
 * Sequence number (1 byte)
 * 
 * PAN ID (2 bytes)
 * 
 * Destination address (1 byte)
 * Destination of this message.
 * 
 * Source address (1 byte)
 * Source of this message.
 * 
 * Message type (1 byte)
 * Indicates the type of message.
 * 0x0 - Discovery type 
 * 0x1 - Synchronization type
 * 0x2 - Ranging type
 *  
 * Reserved (2 bytes)
 * 
 */
extern uint8 def_header[HEADER_LEN];

enum message_type_t peek_msg_type(uint8* msg);
uint16 get_header_id(uint8* header_buf);
uint8 append_header(
  uint8* ret, 
  uint8* payload, 
  uint16 payload_len, 
  uint8 seq_num,
  uint8 src_id, 
  uint8 dest_id, 
  enum message_type_t msg_type);

#endif _HEADER_TEMPLATE_H