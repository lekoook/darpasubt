#include "deca_types.h"
#include "stdio.h"

#ifndef COMMON_H
#define COMMON_H
#include "common.h"
#endif

static void printDists(double dists[NODES], uint8 thisId);
static void printTemp(double temp);
void printData(double dists[NODES], double temp, uint8 thisId);
void printConfig(
  uint8 nodeCount, 
  uint8 thisId, 
  uint16 rngFreq, 
  uint32 txInterval,
  uint32 rxToTxBuffer,
  uint32 cyclePeriod,
  uint32 activePeriod,
  uint32 sleepPeriod,
  uint32 wakeBuffer
  );
