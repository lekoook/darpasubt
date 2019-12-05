#include "deca_types.h"
#include "stdlib.h"

struct VState
{
  uint8 is_new;
  uint8 degree;
  uint16 state;
};

struct Node
{
  uint8 nodes_num;
  uint8 id;
  struct VState *v_states;
};

void Node_constructor(struct Node *node, uint8 id, uint8 nodes_num);
void Node_destructor(struct Node *node);
uint16 Node_pack(struct Node *node, uint8 *packed_data);
void Node_unpack(struct Node *node, uint8 *packed_data, uint8 nodes_num);
void Node_update(struct Node *existing, struct Node *change);
void Node_remove(struct Node *node, uint8 state_id, uint8 to_clear);
uint8 Node_elect(struct Node *node, uint8 ignore_itself);

static uint8 count_set_bits_16(uint16 value);