#include "header_template.h"

uint8 def_header[HEADER_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0x0,  0x0, 0x0, 0x0};

/**
 * @brief Returns the message type given the bytes array.
 * 
 * @param msg pointer to the msg bytes array.
 * @return enum message_type_t the message type. Returns DISC_TYPE, SYNC_TYPE or RNG_TYPE.
 */
enum message_type_t peek_msg_type(uint8* msg)
{
  return msg[IDX_TYPE];
}

/**
 * @brief Returns the node ID from the header.
 * 
 * @param header_buf pointer to the header bytes array.
 * @return uint16 the node ID.
 */
uint16 get_header_id(uint8* header_buf)
{
  return header_buf[IDX_SRC];
}

/**
 * @brief Appends a payload array with the header bytes with the given parameters.
 * 
 * @param ret pointer to a uint8 buffer array to store the resulting array appended with header.
 * @param payload pointer to the uint8 buffer array containing the payload data bytes.
 * @param payload_len length (in bytes) of the payload data bytes. 
 * @param seq_num sequence number to set in header.
 * @param src_id source identifier address of the header.
 * @param dest_id destination identifier address of the header.
 * @param msg_type message type of the header.
 * @return uint8 length (in bytes) of the resulting array.
 */
uint8 append_header(
  uint8* ret, 
  uint8* payload, 
  uint16 payload_len,
  uint8 seq_num,
  uint8 src_id, 
  uint8 dest_id, 
  enum message_type_t msg_type)
{
  uint8 ret_len = (uint16)HEADER_LEN + payload_len;
  memcpy(ret, def_header, HEADER_LEN);
  ret[IDX_SEQ] = seq_num;
  ret[IDX_SRC] = src_id;
  ret[IDX_DEST] = dest_id;
  ret[IDX_TYPE] = (uint8)msg_type;
  memcpy(&(ret[IDX_PAYLOAD]), payload, payload_len);

  return ret_len;
}