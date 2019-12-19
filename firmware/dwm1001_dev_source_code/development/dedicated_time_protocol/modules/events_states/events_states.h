#ifndef _EVENTS_STATES_H
#define _EVENTS_STATES_H

#include "deca_types.h"
#include "deca_device_api.h"
#include "stdbool.h"
#include "common.h"
#include "message_template.h"
#include "timestamper.h"
#include "message_transceiver.h"
#include "int_handler.h"
#include "seq_manager.h"

#define SPEED_OF_LIGHT 299702547
#define OTP_ANT_DLY 0x01C

enum NodeFunction
{
  MASTER_FUNCTION,
  SLAVE_FUNCTION
};

enum state_t
{
  PREPARE_LISTEN_STATE = 0,
  LISTEN_FOR_MASTER_STATE = 1,
  WAIT_RX_PHASE_ZERO_EN_STATE = 2,
  WAIT_RX_PHASE_ZERO_STATE = 3,
  SET_FIRST_TX_STATE = 4,
  WAIT_FIRST_TX_STATE = 5,
  WAIT_RX_PHASE_ONE_STATE = 6,
  SET_SECOND_TX_STATE = 7,
  WAIT_SECOND_TX_STATE = 8,
  WAIT_RX_PHASE_TWO_STATE = 9,
  SLEEP_STATE = 10,
  WAKE_STATE = 11
};

enum event_t
{
  NO_EVENT,
  EN_RX_SUCCESS_EVENT,
  EN_RX_FAIL_EVENT,
  NODE_IS_MASTER_EVENT,
  HEARD_MASTER_TX_EVENT,
  TIMEOUT_EVENT,
  CONFIG_SUCCESS_EVENT,
  CONFIG_FAIL_EVENT,
  TX_SUCCESS_EVENT,
  TX_FAIL_EVENT,
  RECEIVED_ONE_EVENT,
  RECEIVED_ALL_EVENT,
  SLEEP_TIME_PASS_EVENT,
  TX_RX_READY_EVENT
};

struct state_data_t
{
  // Add any data to be used by each state.
  // Time related
  uint32 cycle_period;
  uint32 active_period;
  uint32 sleep_period; // Time duration for actual hardware sleeping
  uint32 wake_period; // Time duration from the last TX/RX to the start of next cycle
  uint32 rx_timeout_1;
  uint32 rx_timeout_2;
  uint16 ant_delay; // Once set, must NEVER be changed.
  uint64 var_delay;
  uint64 reg_delay;

  // States related
  enum NodeFunction node_function;

  // Range variables
  // msg_template is the entire frame to tx out. there is a frame format 
  // (first 10 bytes and last 2 bytes) to follow, check the dw1000 manual.
  msg_template tx_msg_1;
  msg_template tx_msg_2;
  uint8 master_id;
  uint64 ts_table[NUM_STAMPS_PER_CYCLE][NODES];
};

struct state_machine_t
{
  enum state_t state;
  enum event_t event;
  struct state_data_t data;
  void (* state_func)(struct state_machine_t* state_machine);
};

extern bool handle_tx_int;
extern bool handle_rx_int;
extern uint8 header[HEADER_LEN];

/* States function prototypes */
void prepare_listen(struct state_machine_t* state_machine);
void listen_for_master(struct state_machine_t* state_machine);
void wait_rx_phase_zero_en(struct state_machine_t* state_machine);
void wait_rx_phase_zero(struct state_machine_t* state_machine);
void set_first_tx(struct state_machine_t* state_machine);
void wait_first_tx(struct state_machine_t* state_machine);
void wait_rx_phase_one(struct state_machine_t* state_machine);
void set_second_tx(struct state_machine_t* state_machine);
void wait_second_tx(struct state_machine_t* state_machine);
void wait_rx_phase_two(struct state_machine_t* state_machine);
void sleep(struct state_machine_t* state_machine);
void wake(struct state_machine_t* state_machine);

/* Helper function prototypes */
void init_ranging(struct state_machine_t* state_machine);
void handle_tx(struct state_machine_t* state_machine);
void handle_rx(struct state_machine_t* state_machine);

static void tx_update(struct state_machine_t* state_machine, uint64 ts);
static void init_cycle_timings(struct state_data_t* state_data);
static void init_tx_msgs(struct state_data_t* state_data);
static void reset_rx_timeout(void);

#endif