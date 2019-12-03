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
  node->v_states = malloc(sizeof(*node->v_states) * nodes_num);
  
  // Initialise all members of VState struct.
  int i;
  for (i = 0; i < nodes_num; i++)
  {
    node->v_states[i].degree = 0;
    node->v_states[i].is_new = 1;
    node->v_states[i].state = 0;
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
  free(sizeof(*node->v_states) * node->nodes_num);
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
  
  for (i = 0; i < nodes_num; i++)
  {
    /*  
      Pack each v_state as 3 bytes chunk.
      Byte 1       : Status bits (first 4 bits) + Degree count (last 4 bits)
      Byte 2 and 3 : v_state check bits. MSB starting from the left.
    */
    uint8 status = 0;
    uint16 state = 0;
    status |= (node->v_states[i].is_new & 0xF) << 4;
    status |= node->v_states[i].degree & 0xF;
    state = node->v_states[i].state;
    
    packed_data[length++] = status;
    packed_data[length++] = (state & 0xFF00) >> 8;
    packed_data[length++] = (state & 0x00FF);
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

  for (i = 0; i < nodes_num; i++)
  {
    uint8 status = packed_data[index++];
    uint16 state = 0;
    state |= packed_data[index++] << 8;
    state |= packed_data[index++];

    node->v_states[i].is_new = (status & 0xF0) >> 4;
    node->v_states[i].degree = status & 0xF;
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
      existing->v_states[i].state = incoming.state;
    }
    else
    {
      // The incoming degree is lesser, hence it is a credible source. We update with the incoming
      // visibility state information and increment its degree by one.
      existing->v_states[i].state = incoming.state;
      existing->v_states[i].degree++;
    }
  }

  // Update the local visibility state.
  existing->v_states[existing->id].state |= 1 << change->id;
  existing->v_states[existing->id].is_new = 0;
}