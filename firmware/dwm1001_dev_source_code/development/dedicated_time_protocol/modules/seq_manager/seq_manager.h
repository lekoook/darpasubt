#ifndef _SEQ_MANAGER_H
#define _SEQ_MANAGER_H

#include "common.h"
#include "deca_types.h"
#include "string.h"

enum
{
  SEQ_CNT = NODES * 2
};

void set_seq(uint8 seq[SEQ_CNT], uint8 cnt, uint8 curr_id);
void set_to_offset(uint32 interval, uint32 to_buf);
void set_txrx_offset(uint32 interval);

#endif