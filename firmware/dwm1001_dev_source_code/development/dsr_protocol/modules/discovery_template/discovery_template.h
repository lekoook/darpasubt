#ifndef _DISCOVERY_TEMPLATE_H
#define _DISCOVERY_TEMPLATE_H

#include "header_template.h"
#include "v_state.h"
#include "state_machine_def.h"

enum
{
  DISC_MSG_LEN = HEADER_LEN + V_STATES_LEN + CRC_LEN // header + vstates + crc
};

void disc_read_msg(uint8 buffer[DISC_MSG_LEN], struct Node* current);
bool disc_write_msg(uint8 buffer[DISC_MSG_LEN], struct Node* current, uint8 src, uint8 dest);

#endif _DISCOVERY_TEMPLATE_H