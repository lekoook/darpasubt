#ifndef _V_STATE_H
#define _V_STATE_H

#include "deca_types.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "common.h"

struct VState
{
  // TODO: is_new can be removed and represented by state == 0,
  // since every node must know itself, the state is always > 0.
  // Thus, state == 0 is a unique value and we can use that.
  uint8 is_new;
  uint8 degree;
  uint32 state;
};

enum
{
  MISS_COUNT_LIMIT = 10,
  CHANGE_COUNT_LIMIT = 10,
  STABLE_COUNT_LIMIT = 10,
  V_STATE_UNIT_LEN = 6,
  CHNG_CNT_LEN = 2,
  V_STATES_LEN = (NODES * V_STATE_UNIT_LEN) + CHNG_CNT_LEN
};

struct Node
{
  uint8 nodes_num;
  uint8 id;
  struct VState v_states[MAX_NODES];
  uint16 miss_counts[MAX_NODES];
  uint16 change_counts[MAX_NODES];
  uint16 stable_counts;
};

void Node_constructor(struct Node *node, uint8 id, uint8 nodes_num);
void Node_destructor(struct Node *node);
uint16 Node_pack(struct Node *node, uint8 *packed_data);
void Node_unpack(struct Node *node, uint8 *packed_data, uint8 nodes_num);
bool Node_update(struct Node *existing, struct Node *change);
bool Node_remove(struct Node *node, uint8 state_id, uint8 to_clear);
uint8 Node_elect(struct Node *node, uint8 ignore_itself);
bool Node_add_miss_count(struct Node *node);
void Node_reset_miss_count(struct Node *node, uint8 to_clear);
void Node_add_change_count(struct Node *node);
void Node_reset_change_count(struct Node *node);
void Node_update_stable_count(struct Node *node);
void Node_reset_stable_count(struct Node *node);
bool Node_check_stable_count(struct Node *node);

static uint8 count_set_bits_16(uint16 value);
static uint8 count_set_bits_64(uint64 value);

#endif