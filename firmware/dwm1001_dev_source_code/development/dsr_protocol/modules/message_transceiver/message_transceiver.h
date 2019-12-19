#ifndef _MESSAGE_TRANSCEIVER_H
#define _MESSAGE_TRANSCEIVER_H

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

typedef enum {
  TX_SUCCESS,
  TX_ERROR
} TxStatus;

typedef enum {
  RX_SUCCESS,
  RX_ERROR,
} RxStatus;

TxStatus txMsg(uint8 *msg, int msgLen, uint8 mode);
RxStatus rxMsg(uint8 *buffer);

#endif