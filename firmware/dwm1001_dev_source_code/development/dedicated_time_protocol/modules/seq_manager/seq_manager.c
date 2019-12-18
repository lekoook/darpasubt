#include "seq_manager.h"

static uint8 id = 0;
static uint8 nodes_count = 0;
static int16 curr_ptr = -1;
static uint64 ref_time = 0;
static uint8 rx_zero_cnt = 0;
static uint8 rx_one_cnt = 0;
static uint8 rx_two_cnt = 0;
static uint8 rx_zero_cnt_max = 0;
static uint8 rx_one_cnt_max = 0;
static uint8 rx_two_cnt_max = 0;
static uint8 sequence[SEQ_CNT] = {0};
static uint8 sequence_flag[SEQ_CNT] = {0};
static uint32 to_offset[SEQ_CNT] = {0};
static uint32 txrx_offset[SEQ_CNT] = {0};

/**
 * @brief Sets the sequence of node.
 * 
 * @param seq pointer to array of node IDs representing the sequence.
 */
void set_seq(uint8 seq[SEQ_CNT], uint8 cnt, uint8 curr_id)
{
  memcpy(sequence, seq, sizeof(*sequence) * cnt);
  memcpy(sequence_flag, seq, sizeof(*sequence_flag) * cnt);
  id = curr_id;
  nodes_count = cnt;

  int i = 0;
  for (; i < cnt; i++)
    if (sequence[i] == id)
      break;
  rx_zero_cnt_max = i;
  i++;

  for (; i < cnt; i++)
    if (sequence[i] == id)
      break;
  rx_one_cnt_max = i - rx_zero_cnt_max - 1;
  rx_two_cnt_max = SEQ_CNT - i - 1;

  reset_cnt();
}

/**
 * @brief Sets the timeout offsets for each node in the sequence.
 * 
 * @param interval microseconds interval between two nodes.
 * @param to_buf microseconds timeout buffer from a node's transmission/reception time stamp.
 */
void set_to_offset(uint32 interval, uint32 to_buf)
{
  int i;
  for (i = 0; i < SEQ_CNT; i++)
  {
    to_offset[i] = interval * i;
    if (i > 0) 
      to_offset[i] += to_buf;
  }
}

/**
 * @brief Sets the TX/RX offsets for each node in the sequence.
 * 
 * @param interval microseconds interval between two nodes.
 */
void set_txrx_offset(uint32 interval)
{
  int i;
  for (i = 0; i < SEQ_CNT; i++)
  {
    txrx_offset[i] = interval * i;
  }
}

uint8 next_node(void)
{
  return sequence[curr_ptr + 1];
}

void update_rx_zero(void)
{
  rx_zero_cnt++;
}

void update_rx_one(void)
{
  rx_one_cnt++;
}

void update_rx_two(void)
{
  rx_two_cnt++;
}

int rx_zero_complete(void)
{
  return rx_zero_cnt == rx_zero_cnt_max;
}

int rx_one_complete(void)
{
  return rx_one_cnt == rx_one_cnt_max;
}

int rx_two_complete(void)
{
  return rx_two_cnt == rx_two_cnt_max;
}

void reset_cnt(void)
{
  rx_zero_cnt = 0;
  rx_one_cnt = 0;
  rx_two_cnt = 0;
}