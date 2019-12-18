#ifndef _MESSAGE_TRANSCEIVER_H
#define _MESSAGE_TRANSCEIVER_H

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "message_template.h"

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
void writeTx2(msg_template *msg, uint64 tsTable[NUM_STAMPS_PER_CYCLE][NODES]);
bool configTx(
  uint64 tsTable[NUM_STAMPS_PER_CYCLE][NODES],
  uint64 txDelay,
  uint64 refTs,
  msg_template *txMsg2
  );
TxStatus convertTx(msg_template *msg, uint8 mode);

#endif