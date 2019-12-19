#ifndef _STATE_MACHINE_H
#define _STATE_MACHINE_H

#include "deca_types.h"
#include "common.h"
#include "low_timer.h"
#include "v_state.h"

enum NodeFunction
{
  MASTER_FUNCTION,
  SLAVE_FUNCTION
};

enum state_t
{
  DISC_LISTEN_STATE,
  DISC_BROADCAST_STATE,
  SLEEP_STATE
};

enum event_t
{
  NO_EVENT,
  FAIL_EVENT,
  HEARD_DISC_MSG_EVENT,
  HEARD_NON_DISC_MSG_EVENT,
  SILENT_EVENT,
  STABLE_EVENT,
  WAKE_EVENT
};

struct state_data_t
{
  enum NodeFunction node_function;
  uint32 listen_to_time;
  uint16 ant_delay; // Once set, must NEVER be changed.
  uint64 var_delay;
  uint64 reg_delay;

  const app_timer_id_t* listen_timer;

  struct Node disc_info;
};

struct state_machine_t
{
  enum state_t state;
  enum event_t event;
  struct state_data_t data;
  void (* state_func)(struct state_machine_t* state_machine);
};

#endif
