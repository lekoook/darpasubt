#include "events_states.h"

bool handle_tx_int = false;
bool handle_rx_int = false;
uint8 header[HEADER_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0};

///////////////////////////////////////////////////////////
// State Functions
///////////////////////////////////////////////////////////

/**
 * @brief Turns on the receiver.
 * 
 * @param state_machine state_machine_t struct object representing the state machine.
 */
void prepare_listen(struct state_machine_t* state_machine)
{
  
  if (state_machine->data.node_function == MASTER_FUNCTION)
  {
    state_machine->event = NO_EVENT;
    state_machine->state = SET_FIRST_TX_STATE;
    state_machine->state_func = set_first_tx;
    return;
  }

  if (state_machine->event == EN_RX_SUCCESS_EVENT)
  {
    // Transition to listen_for_master state.
    state_machine->event = NO_EVENT;
    state_machine->state = LISTEN_FOR_MASTER_STATE;
    state_machine->state_func = listen_for_master;
    return;
  }

  if (state_machine->event == EN_RX_FAIL_EVENT)
  {
    // Transition to self state.
    state_machine->event = NO_EVENT;
    state_machine->state = PREPARE_LISTEN_STATE;
    state_machine->state_func = prepare_listen;
    return;
  }
  
  if (dwt_rxenable(DWT_START_RX_IMMEDIATE) == DWT_SUCCESS)
  {
    state_machine->event = EN_RX_SUCCESS_EVENT;
  }
  else
  {
    state_machine->event = EN_RX_FAIL_EVENT;
  }
}

/**
 * @brief Waits for the reception of master TX. Transition to next state immediately if node is master.
 * 
 * @param state_machine state_machine_t struct object representing the state machine.
 */
void listen_for_master(struct state_machine_t* state_machine)
{
  if (state_machine->event == HEARD_MASTER_TX_EVENT)
  {
    state_machine->event = NO_EVENT;
    state_machine->state = WAIT_RX_PHASE_ZERO_STATE;
    state_machine->state_func = wait_rx_phase_one;
    return;
  }

  // Do nothing as we are just waiting for master TX.
}

void wait_rx_phase_zero(struct state_machine_t* state_machine)
{
  if (state_machine->event == RECEIVED_ALL_EVENT || state_machine->event == TIMEOUT_EVENT)
  {
    state_machine->event = NO_EVENT;
    state_machine->state = SET_FIRST_TX_STATE;
    state_machine->state_func = set_first_tx;
    return;
  }


}

void set_first_tx(struct state_machine_t* state_machine)
{

}

void wait_first_tx(struct state_machine_t* state_machine)
{

}

void wait_rx_phase_one(struct state_machine_t* state_machine)
{

}

void set_second_tx(struct state_machine_t* state_machine)
{

}

void wait_second_tx(struct state_machine_t* state_machine)
{

}

void wait_rx_phase_two(struct state_machine_t* state_machine)
{

}

void sleep(struct state_machine_t* state_machine)
{

}

void wake(struct state_machine_t* state_machine)
{

}

///////////////////////////////////////////////////////////
// State Functions - end
///////////////////////////////////////////////////////////

/**
 * @brief Initialises all variables required for ranging process.
 * 
 */
void init_ranging(struct state_machine_t* state_machine)
{
  // Initialise all state machine variables.
  state_machine->state = PREPARE_LISTEN_STATE;
  state_machine->event = NO_EVENT;
  state_machine->state_func = prepare_listen;
  
  // TODO: Temporary solution to assign function status.
  if (NODE_ID == 0) state_machine->data.node_function = MASTER_FUNCTION;
  else state_machine->data.node_function = SLAVE_FUNCTION;
  
  // Pre-calculate all the timings in one cycle (ie, cycle, active, sleep period).
  init_cycle_timings(&(state_machine->data));

  // Initialise timestamp-related data structures.
  initTable(state_machine->data.ts_table);

  // Initalises the messages to be transmitted.
  init_tx_msgs(&(state_machine->data));

  // Prints configuration information.
  printConfig(
    NODES,
    NODE_ID,
    RANGE_FREQ,
    TX_INTERVAL,
    RX_TX_BUFFER,
    state_machine->data.cycle_period,
    state_machine->data.active_period,
    state_machine->data.sleep_period,
    WAKE_BUFFER
  );
}

void handle_tx(struct state_machine_t* state_machine)
{
  // TODO: implement this
}

/**
 * @brief Handles the reception of messages.
 * 
 * @param state_machine 
 */
void handle_rx(struct state_machine_t* state_machine)
{
  handle_rx_int = false; // Reset the flag.
  uint8 buffer[MSG_LEN] = {0};

  if (rxMsg(buffer) != RX_SUCCESS)
  {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return;
  }

  // Reception succeed, now we want to process according the State.  
  msg_template msg;
  convertToStruct(buffer, &msg);

  switch (state_machine->state)
  {
  case LISTEN_FOR_MASTER_STATE:
    if (msg.id == state_machine->data.master_id &&
        msg.isFirst == 1 &&
        state_machine->data.node_function == SLAVE_FUNCTION)
    {
      // This is the initial message from master we are waiting for, update the Event.
      state_machine->event = HEARD_MASTER_TX_EVENT;
      // Update the record table with the timestamp of reception.
      uint64 ts = getRxTimestampU64();
      updateTable(state_machine->data.ts_table, msg, ts, NODE_ID);
    }
    break;
  
  case WAIT_RX_PHASE_ZERO_STATE:
    ; // Prevent label can only be part of a statement error.
    uint64 ts = getRxTimestampU64();
    updateTable(state_machine->data.ts_table, msg, ts, NODE_ID);
    uint16_t timeout = get_next_timeout();
    dwt_setrxtimeout(timeout);
    // if (rx_zero_complete())
    // {
    //   state_machine->event = RECEIVED_ALL_EVENT;
    // }
    // else
    // {
    //   state_machine->event = RECEIVED_ONE_EVENT;
    // }
    
    break;

  case WAIT_RX_PHASE_ONE_STATE:
    break;

  case WAIT_RX_PHASE_TWO_STATE:
    break;
  
  default:
    break;
  }

  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/**
 * @brief Updates first TX timestamp in timestamp table and start timer for second TX.
 * 
 * @param ts timestamp of first TX.
 */
static void tx_update(struct state_machine_t* state_machine, uint64 ts)
{
  updateTable(state_machine->data.ts_table, state_machine->data.tx_msg_1, ts, NODE_ID);
}

/**
 * @brief Initialises the cycle timings.
 * 
 * @details These timings are the cycle, active and sleep periods.
 */
static void init_cycle_timings(struct state_data_t* state_data)
{
  uint64 interval;

  state_data->cycle_period = 1000000 / RANGE_FREQ; // Convert from seconds to microseconds.
  state_data->active_period = (2 * NODES * TX_INTERVAL); // Number of intervals in NODES nodes.
  state_data->sleep_period = state_data->cycle_period - state_data->active_period;
  state_data->wake_period = state_data->cycle_period - WAKE_BUFFER;
  
  // RX timeout values for synchronising TX moments.
  state_data->rx_timeout_1 = ((uint32)TX_INTERVAL * (uint32)NODE_ID);
  state_data->rx_timeout_2 = ((uint32)TX_INTERVAL * (uint32)NODES) + state_data->rx_timeout_1;
  // The values are deducted with a fixed value to allow transition time from RX to TX for transceiver.
  if (state_data->rx_timeout_1 > (uint32)RX_TX_BUFFER)
    state_data->rx_timeout_1 -= (uint32)RX_TX_BUFFER;
  else
    state_data->rx_timeout_1 = 0;

  if (state_data->rx_timeout_2 > (uint32)RX_TX_BUFFER)
    state_data->rx_timeout_2 -= (uint32)RX_TX_BUFFER;
  else
    state_data->rx_timeout_2 = 0;
  
  interval = (uint64)TX_INTERVAL * (uint64)UUS_TO_DWT_TIME; // interval in DWT time units
  uint64 temp = 500; // TODO: We need this to allow TX2 to be configured early enough. However, this will push back the TX2 and potentially mess up with the next node attempting to receive before switching from RX to TX. Investigate.
  state_data->reg_delay = (NODES * interval) + (temp * (uint64)UUS_TO_DWT_TIME);
  state_data->var_delay = (NODE_ID * interval);
}

/**
 * @brief Initialises the TX messages to be used during the cycle.
 * 
 */
static void init_tx_msgs(struct state_data_t* state_data)
{
  state_data->tx_msg_1.id = NODE_ID;
  state_data->tx_msg_1.isFirst = 1;
  memcpy(state_data->tx_msg_1.header, header, HEADER_LEN);
  memset(state_data->tx_msg_1.data, 0, DATA_LEN);
  memset(state_data->tx_msg_1.crc, 0, CRC_LEN);

  state_data->tx_msg_2.id = NODE_ID;
  state_data->tx_msg_2.isFirst = 0;
  memcpy(state_data->tx_msg_2.header, header, HEADER_LEN);
  memset(state_data->tx_msg_2.data, 0, DATA_LEN);
  memset(state_data->tx_msg_2.crc, 0, CRC_LEN);
}