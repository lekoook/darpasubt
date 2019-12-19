#ifndef _TIMESTAMPER_H
#define _TIMESTAMPER_H

#include "deca_device_api.h"
#include "port_platform.h"

uint64 getTxTimestampU64(void);
uint64 getRxTimestampU64(void);
uint64 getSysTimeU64(void);

#endif