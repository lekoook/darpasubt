#include "events_states.h"

bool handle_tx_int = false;
bool handle_rx_int = false;
struct state_machine_t* dsr_state_machine;
uint64 time1;
uint64 time2;
///////////////////////////////////////////////////////////
// State Functions
///////////////////////////////////////////////////////////
// APP_TIMER_DEF(listen_timer);
/**
 * @brief Entry function for Discovery Listen state.
 * @details Turns on the receiver and begin the watchdog timer for listening to messages.
 * 
 * @param state_machine pointer to struct representing the state machine.
 */
void disc_listen_en(struct state_machine_t *state_machine)
{
  time1 = getSysTimeU64();
  lowTimerStart(*(state_machine->data.listen_timer), state_machine->data.listen_to_time, state_machine);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  state_machine->state_func = disc_listen;
}

/**
 * @brief Body function for Discovery Listen state.
 * @details Checks and take actions for the following events:
 * - Heard discovery message: Stop listening timer and transit to Discovery Broadcast state.
 * - Heard non-discovery message: Stop listening timer and transit to Sleep state.
 * - Nothing heard: Transit to Discovery Broadcast state.
 * 
 * @param state_machine pointer to struct representing the state machine.
 */
void disc_listen(struct state_machine_t *state_machine)
{
  if (state_machine->event == HEARD_DISC_MSG_EVENT)
  {
    state_machine->event = NO_EVENT;
    state_machine->state = DISC_BROADCAST_STATE;
    state_machine->state_func = disc_broadcast_en;
    return;
  }

  if (state_machine->event == HEARD_NON_DISC_MSG_EVENT)
  {
    state_machine->event = NO_EVENT;
    state_machine->state = SLEEP_STATE;
    state_machine->state_func = sleep;
    return;
  }

  if (state_machine->event == SILENT_EVENT)
  {
    state_machine->event = NO_EVENT;
    state_machine->state = DISC_BROADCAST_STATE;
    state_machine->state_func = disc_broadcast_en;
    return;
  }
}

void disc_broadcast_en(struct state_machine_t* state_machine)
{
  state_machine->state_func = disc_broadcast;
  ren_rx_w_to();
}

void disc_broadcast(struct state_machine_t* state_machine)
{
  if (state_machine->event == STABLE_EVENT)
  {
    state_machine->event = NO_EVENT;
    state_machine->state_func = disc_broadcast_ex;
  }
  if (state_machine->event == FAIL_EVENT)
  {
    state_machine->event = NO_EVENT;
    state_machine->state = SLEEP_STATE;
    state_machine->state_func = sleep;
  }
}

void disc_broadcast_ex(struct state_machine_t* state_machine)
{
  printf("exit\r\n");
  state_machine->state = SLEEP_STATE;
  state_machine->state_func = sleep;
  printf("id: %u\r\n", state_machine->data.disc_info.id);
  int i;
  for (i = 0; i < state_machine->data.disc_info.nodes_num; i++)
  {
    printf("%d: %d\r\n",i, state_machine->data.disc_info.v_states[i].state);
  }
}

void sleep(struct state_machine_t* state_machine)
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
  dsr_state_machine = state_machine;
  
  // Initialise all state machine variables.
  state_machine->state = DISC_LISTEN_STATE;
  state_machine->event = NO_EVENT;
  state_machine->state_func = disc_listen_en;
  
  // TODO: Temporary solution to assign function status.
  if (NODE_ID == 0) 
    state_machine->data.node_function = MASTER_FUNCTION;
  else 
    state_machine->data.node_function = 1;

  init_timers(state_machine);

  Node_constructor(&(state_machine->data.disc_info), NODE_ID, NODES);

  // Initialise the Random Number Generator module.
  nrf_drv_rng_init(NULL);

  // // Pre-calculate all the timings in one cycle (ie, cycle, active, sleep period).
  // init_cycle_timings(&(state_machine->data));

  // Initialise timestamp-related data structures.
  // initTable(state_machine->data.ts_table);

  // // Initalises the messages to be transmitted.
  // init_tx_msgs(&(state_machine->data));

  // Prints configuration information.
  // printConfig(
  //   NODES,
  //   NODE_ID,
  //   RANGE_FREQ,
  //   TX_INTERVAL,
  //   RX_TX_BUFFER,
  //   state_machine->data.cycle_period,
  //   state_machine->data.active_period,
  //   state_machine->data.sleep_period,
  //   WAKE_BUFFER
  // );
}

void handle_tx(void)
{
  switch (dsr_state_machine->state)
  {
  case DISC_BROADCAST_STATE:
    ren_rx_w_to();
    break;
  
  default:
    break;
  }
}

/**
 * @brief Handles the reception of messages.
 * 
 */
void handle_rx(void)
{
  handle_rx_int = false; // Reset the flag.
  uint8 buffer[DISC_MSG_LEN] = {0};

  if (rxMsg(buffer) != RX_SUCCESS)
  {
    printf("RX read error\r\n");
    return;
  }

  // Reception succeed, now we want to process according to the state.  
  enum message_type_t msg_type = peek_msg_type(buffer);
  switch (dsr_state_machine->state)
  {
  case DISC_LISTEN_STATE:
    if (msg_type == DISC_TYPE)
    {
      lowTimerStop(*(dsr_state_machine->data.listen_timer));
      dsr_state_machine->event = HEARD_DISC_MSG_EVENT;
      disc_read_msg(buffer, &(dsr_state_machine->data.disc_info));
    }

    if (msg_type != DISC_TYPE)
    {
      lowTimerStop(*(dsr_state_machine->data.listen_timer));
      dsr_state_machine->event = HEARD_NON_DISC_MSG_EVENT;
    }
    break;

  case DISC_BROADCAST_STATE:
    if (msg_type == DISC_TYPE)
    {
      disc_read_msg(buffer, &(dsr_state_machine->data.disc_info));
      printf("1");
      ren_rx_w_to();
    }
  
  default:
    break;
  }
}

void handle_rx_to(void)
{
  switch (dsr_state_machine->state)
  {
  case DISC_BROADCAST_STATE:
    ;
    uint8 buf[DISC_MSG_LEN] = {0};
    bool res = disc_write_msg(buf, &(dsr_state_machine->data.disc_info), NODE_ID, 0);
    if (txMsg(buf, DISC_MSG_LEN, DWT_START_TX_IMMEDIATE) != TX_SUCCESS)
    {
      dsr_state_machine->event = FAIL_EVENT;
    }
    printf("2");
    if (res) 
      dsr_state_machine->event = STABLE_EVENT;
    break;
  
  default:
    break;
  }
}

/**
 * @brief Resets and re-enables the receiver with a randomly generated timeout time.
 * 
 */
void ren_rx_w_to(void)
{
  // Ensure transceiver is off before setting RX timeout. Undefined behaviour if not set at idle mode.
  dwt_forcetrxoff();
  uint16 timeout = ((double)(get_rand_num(sizeof(timeout)))) / 1.0256;
  dwt_setrxtimeout(timeout);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

// /**
//  * @brief Updates first TX timestamp in timestamp table and start timer for second TX.
//  * 
//  * @param ts timestamp of first TX.
//  */
// static void tx_update(struct state_machine_t* state_machine, uint64 ts)
// {
//   updateTable(state_machine->data.ts_table, state_machine->data.tx_msg_1, ts, NODE_ID);
// }

// /**
//  * @brief Initialises the cycle timings.
//  * 
//  * @details These timings are the cycle, active and sleep periods.
//  */
// static void init_cycle_timings(struct state_data_t* state_data)
// {
//   uint64 interval;

//   state_data->cycle_period = 1000000 / RANGE_FREQ; // Convert from seconds to microseconds.
//   state_data->active_period = (2 * NODES * TX_INTERVAL); // Number of intervals in NODES nodes.
//   state_data->sleep_period = state_data->cycle_period - state_data->active_period;
//   state_data->wake_period = state_data->cycle_period - WAKE_BUFFER;
  
//   // RX timeout values for synchronising TX moments.
//   state_data->rx_timeout_1 = ((uint32)TX_INTERVAL * (uint32)NODE_ID);
//   state_data->rx_timeout_2 = ((uint32)TX_INTERVAL * (uint32)NODES) + state_data->rx_timeout_1;
//   // The values are deducted with a fixed value to allow transition time from RX to TX for transceiver.
//   if (state_data->rx_timeout_1 > (uint32)RX_TX_BUFFER)
//     state_data->rx_timeout_1 -= (uint32)RX_TX_BUFFER;
//   else
//     state_data->rx_timeout_1 = 0;

//   if (state_data->rx_timeout_2 > (uint32)RX_TX_BUFFER)
//     state_data->rx_timeout_2 -= (uint32)RX_TX_BUFFER;
//   else
//     state_data->rx_timeout_2 = 0;
  
//   interval = (uint64)TX_INTERVAL * (uint64)UUS_TO_DWT_TIME; // interval in DWT time units
//   uint64 temp = 500; // TODO: We need this to allow TX2 to be configured early enough. However, this will push back the TX2 and potentially mess up with the next node attempting to receive before switching from RX to TX. Investigate.
//   state_data->reg_delay = (NODES * interval) + (temp * (uint64)UUS_TO_DWT_TIME);
//   state_data->var_delay = (NODE_ID * interval);
// }

// /**
//  * @brief Initialises the TX messages to be used during the cycle.
//  * 
//  */
// static void init_tx_msgs(struct state_data_t* state_data)
// {
//   state_data->tx_msg_1.id = NODE_ID;
//   state_data->tx_msg_1.isFirst = 1;
//   memcpy(state_data->tx_msg_1.header, header, HEADER_LEN);
//   memset(state_data->tx_msg_1.data, 0, DATA_LEN);
//   memset(state_data->tx_msg_1.crc, 0, CRC_LEN);

//   state_data->tx_msg_2.id = NODE_ID;
//   state_data->tx_msg_2.isFirst = 0;
//   memcpy(state_data->tx_msg_2.header, header, HEADER_LEN);
//   memset(state_data->tx_msg_2.data, 0, DATA_LEN);
//   memset(state_data->tx_msg_2.crc, 0, CRC_LEN);
// }

// static void reset_rx_timeout(void)
// {
//   dwt_forcetrxoff();
  
// //  dwt_setrxtimeout();
// }

/**
 * @brief Initialises all required timers.
 * 
 * @param state_machine 
 */
void init_timers(struct state_machine_t* state_machine)
{
  lowTimerInit();
  state_machine->data.listen_to_time = LISTEN_TIMEOUT_TIME;
  lowTimerSingleCreate(state_machine->data.listen_timer, listen_to_handler); 
}

/**
 * @brief Callback handler that executes when Discovery Listen state's watchdog timer expires.
 * 
 * @param state_machine pointer to struct representing the state machine, passed in when the timer was started.
 */
void listen_to_handler(struct state_machine_t* state_machine)
{
  state_machine->event = SILENT_EVENT;
}

/**
 * @brief Get a random number using the nRF Random Number Generator (RNG) module.
 * 
 * @param bytes_to_read the number of bytes to read from the module.
 * @return uint64 randomly generated 64 bit unsigned integer.
 */
static uint64 get_rand_num(uint8 bytes_to_read)
{
  uint8 buf[8] = {0};
  uint8 bytesAvail = 0;
  uint64 ret = 0;
  int i;
  
  // Get the random generated bytes from the module.
  do
  {
    nrf_drv_rng_bytes_available(&bytesAvail);
  } while (bytesAvail < bytes_to_read); // Wait until there are enough generated random bytes.
  
  // Read and return the random generated number.
  nrf_drv_rng_rand(buf, bytes_to_read);
  for (i = 0; i < bytes_to_read; i++)
  {
    ret |= buf[i];
    ret <<= 8;
  }

  return ret;
}