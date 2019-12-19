#include "v_state.h"

/**
 * @brief Constructs the Node struct with default values.
 * 
 * @param node pointer to Node struct to initialise.
 * @param id identification number of the node.
 * @param nodes_num total number of nodes.
 */
void Node_constructor(struct Node *node, uint8 id, uint8 nodes_num)
{
  // Initialise all members.
  node->nodes_num = nodes_num;
  node->id = id;
  
  // Initialise all members of VState structs and counters.
  int i;
  for (i = 0; i < nodes_num; i++)
  {
    node->v_states[i].degree = 0;
    node->v_states[i].is_new = 1;
    node->v_states[i].state = 0;

    node->miss_counts[i] = 0;
    node->change_counts[i] = 0;
    node->stable_counts[i] = 0;
  }
  node->v_states[id].state = 1 << id;
}

/**
 * @brief Destructs the Node struct and free memory.
 * 
 * @param node pointer to Node struct to destruct.
 */
void Node_destructor(struct Node *node)
{
  // Do nothing.
}

/**
 * @brief Packs a given Node struct into a visibility states information bytes array.
 * 
 * @param node pointer to Node struct to be packed.
 * @param packed_data pointer to array to store packed data.
 * @return uint16 size of array in bytes.
 */
uint16 Node_pack(struct Node *node, uint8 *packed_data)
{
  uint8 i = 0;
  uint8 nodes_num = node->nodes_num;
  uint16 length = 0;

  packed_data[length++] = node->id; // Pack id first.
  packed_data[length++] = (node->miss_counts[node->id] >> 8) & 0xFF; // Higher byte of miss_count
  packed_data[length++] = node->miss_counts[node->id] & 0xFF; // Lower byte of miss_count
  
  for (i = 0; i < nodes_num; i++)
  {
    /*  
      Pack each v_state as 3 bytes chunk.
      Byte 1       : Status bits (first 4 bits) + Degree count (last 4 bits)
      Byte 2 and 3 : v_state check bits. MSB starting from the left.
    */
    packed_data[length++] = node->v_states[i].is_new;
    packed_data[length++] = node->v_states[i].degree;
    
    uint64 state = node->v_states[i].state;
    uint8 j;
    for (j = sizeof(state)-1; j >= 0; i--)
    {
      packed_data[length++] = (state >> (j * 8)) & 0xFF;
    }
  }

  return length;
}

/**
 * @brief Unpacks a visibility states information bytes array into a Node structure.
 * 
 * @param node pointer to Node struct to be unpacked to.
 * @param packed_data pointer to array containing the information bytes.
 */
void Node_unpack(struct Node *node, uint8 *packed_data, uint8 nodes_num)
{
  uint8 i = 0;
  uint16 index = 0;

  node->id = packed_data[index++]; // Unpack id first.
  node->miss_counts[node->id] |= (packed_data[index++] << 8);
  node->miss_counts[node->id] |= packed_data[index++];

  for (i = 0; i < nodes_num; i++)
  {
    node->v_states[i].is_new = packed_data[index++];
    node->v_states[i].degree = packed_data[index++];
    
    uint64 state = 0;
    uint8 j;
    for (j = sizeof(state)-1; j >= 0; j--)
    {
      state |= ((uint64)packed_data[index]) << (j * 8);
    }
    node->v_states[i].state = state;
  }
}

/**
 * @brief Updates the local node's visibility states information with an incoming node's information.
 * 
 * @param existing pointer to the local node's Node struct.
 * @param change pointer to the incoming node's Node struct.
 */
void Node_update(struct Node *existing, struct Node *change)
{
  if (existing->id == change->id) return; // Sanity check.

  // Check for credibility by comparing every visibility states of each tracked node with the
  // incoming node's visibility states.
  int i;
  for (i = 0; i < existing->nodes_num; i++)
  {
    if (i == existing->id) continue; // No one knows more about the local node than itself.
    struct VState current = existing->v_states[i];
    struct VState incoming = change->v_states[i];

    if (incoming.is_new || incoming.degree > current.degree)
    {
      // The incoming visibility state is either new or not credible.
      continue;
    }
    else if (current.is_new && !incoming.is_new)
    {
      // What we have now is new, update with whatever we get from incoming (that is not new).
      existing->v_states[i] = incoming;
      existing->v_states[i].degree++;
    }
    else if (incoming.degree == current.degree)
    {
      // The degree of separation is the same. Consider it credible.
      // But do not increment the degree since there is no difference in the degree between
      // previous value and incoming value.
      existing->v_states[i] = incoming;
    }
    else
    {
      // The incoming degree is lesser, hence it is a credible source. We update with the incoming
      // visibility state information and increment its degree by one.
      existing->v_states[i] = incoming;
      existing->v_states[i].degree++;
    }
  }

  // Update the local visibility state.
  existing->v_states[existing->id].state |= 1 << change->id;
  existing->v_states[existing->id].is_new = 0;
}

/**
 * @brief Remove the occurence of an node id from a visibility state information.
 * 
 * @param node pointer to the local node's Node struct.
 * @param state_id id of the visibility state information to remove from.
 * @param to_clear id to remove in the visibility state information given.
 */
void Node_remove(struct Node *node, uint8 state_id, uint8 to_clear)
{
  node->v_states[state_id].state &= ~(1 << to_clear);
}

/**
 * @brief Elect a leader based on the given visibility states. The node with the highest count is the leader.
 * 
 * @param node pointer to the local node's Node struct.
 * @param ignore_itself boolean flag to ignore the node's own visibility state during checking.
 * @return uint8 the identifier of the selected leader.
 */
uint8 Node_elect(struct Node *node, uint8 ignore_itself)
{
  uint8 selected = 0;
  uint32 highest = 0;

  // Find the first node with the largest number of set bits in their visibility state which means
  // that node sees the most number of other nodes.
  int i;
  for (i = 0; i < node->nodes_num; i++)
  {
    if (ignore_itself && i == node->id) continue;

    uint8 count = count_set_bits_16(node->v_states[i].state);
    if (count > highest)
    {
      highest = count;
      selected = i;
    }
  }

  return selected;
}

/**
 * @brief Count the number of set bits ('1') given a 16 bits unsigned integer.
 * 
 * @param value 16 bits unsigned integer to check.
 * @return uint8 number of bits set to '1'.
 */
static uint8 count_set_bits_16(uint16 value)
{
  // Count the number of set bits for every 2-bits pair, in parallel.
  // Expanding the operation from the most inner bracket scope,
  // 0111 -> 0011 (shift right by 1 bit)
  // 0011 -> 0001 (mask with 0x5555 = 0b0101010101010101)
  // 0001 -> 0110 (subtract from original value: 0111 - 0001 = 0110)
  // Now we get the number of bits in each pair (0, 1 or 2) represented in each pair bits.
  value = value - ((value >> 1) & 0x5555);

  // Similar to above, here finds the sum of every first and second pair of 2-bits, 
  // represented as 4-bits group.
  // Expanding from left and from the most inner bracket scope,
  // 0110 -> 0010 (mask with 0x3333)
  // 0110 -> 0001 (shift right by 2 bits)
  // 0001 -> 0001 (mask with 0x3333)
  // 0010 + 0001 -> 0011 (sum both 2-bits pair into one 4-bits group).
  value = (value & 0x3333) + ((value >> 2) & 0x3333);

  // Finally, we find the final sum of set bits.
  // Expanding from left and from most inner bracket scope, 
  // given 1001 0111 0101 1010 as original value,
  // then the above two operations will give 0010 0011 0010 0010.
  // 0010 0011 0010 0010 -> 0000 0010 0011 0010 (shift right by 4 bits)
  // 0000 0010 0011 0010 -> 0010 0101 0101 0100 (add to previous value)
  // 0010 0101 0101 0100 -> 0000 0101 0000 0100 (mask with 0x0F0F)
  // 0000 0101 0000 0100 -> 0000 1001 0000 0100 (multiply with 0x0101, is same as (num << 8) + num)
  // Now, the higher 8 bits contains the total number of set bits for the entire number.
  // Shift right by 8 bits.
  return (((value + (value >> 4)) & 0x0F0F) * 0x0101) >> 8;
}

static uint8 count_set_bits_64(uint64 value)
{
  // TODO: Implement the 64 bits version.
}