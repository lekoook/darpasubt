#ifndef _EVENTS_STATES_H
#define _EVENTS_STATES_H

// #include "deca_types.h"
#include "deca_device_api.h"
#include "stdbool.h"
// #include "common.h"
#include "low_timer.h"
#include "message_transceiver.h"
#include "discovery_template.h"
#include "state_machine_def.h"
#include "timestamper.h"

#define OTP_ANT_DLY 0x01C

enum
{
  LISTEN_TIMEOUT_TIME = 100000,
  DISC_RX_TIMEOUT = 65001
};

extern bool handle_tx_int;
extern bool handle_rx_int;
extern struct state_machine_t* dsr_state_machine;

/* Discovery states function prototypes */
void disc_listen_en(struct state_machine_t* state_machine);
void disc_listen(struct state_machine_t* state_machine);
// Yet to implement below
void disc_broadcast_en(struct state_machine_t* state_machine);
void disc_broadcast(struct state_machine_t* state_machine);
void disc_broadcast_ex(struct state_machine_t* state_machine);
void sleep(struct state_machine_t* state_machine);

/* Helper function prototypes */
void init_ranging(struct state_machine_t* state_machine);
void handle_tx(void);
void handle_rx(void);
void handle_rx_to(void);
void ren_rx_w_to(void);

// static void tx_update(struct state_machine_t* state_machine, uint64 ts);
// static void init_cycle_timings(struct state_data_t* state_data);
// static void init_tx_msgs(struct state_data_t* state_data);
// static void reset_rx_timeout(void);

/* Timers functions */
void init_timers(struct state_machine_t* state_machine);
void listen_to_handler(struct state_machine_t* state_machine);

static uint64 get_rand_num(uint8 bytes_to_read);

#endif