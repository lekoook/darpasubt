#include "discovery_template.h"

/**
 * @brief Updates the Discovery record given the buffer array containing an incoming broadcase message.
 * 
 * @param buffer pointer to the bytes array.
 * @param current pointer to the local node's Discovery record.
 */
void disc_read_msg(uint8 buffer[DISC_MSG_LEN],  struct Node* current)
{
  struct Node incoming;
  incoming.id = get_header_id(buffer);
  Node_unpack(&incoming, &buffer[IDX_PAYLOAD], current->nodes_num);

  Node_reset_miss_count(current, incoming.id);
  if (Node_update(current, &incoming))
  {
     // There is change in record.
    Node_reset_change_count(current);
    Node_reset_stable_count(current);
  }
}

/**
 * @brief Writes the broadcast message given the current node's Discovery record.
 * 
 * @param buffer pointer to the bytes array to contain the message.
 * @param current pointer to the local node's Discovery record.
 * @param src source identifier address of the node in the header.
 * @param dest destination identifier address of the node in the header.
 * @return true if the local node has achieved complete stability. (All nodes in record has achieved temporary stability.)
 * @return false otherwise.
 */
bool disc_write_msg(uint8 buffer[DISC_MSG_LEN], struct Node* current, uint8 src, uint8 dest)
{
  uint8 v_state[V_STATES_LEN] = {0};
  
  if (Node_add_miss_count(current))
  {
    Node_reset_change_count(current);
    Node_reset_stable_count(current);
  }
  Node_add_change_count(current);
  Node_update_stable_count(current);
  uint16 v_len = Node_pack(current, v_state);
  append_header(buffer, v_state, v_len, 0, src, dest, DISC_TYPE);

  return Node_check_stable_count(current);
}