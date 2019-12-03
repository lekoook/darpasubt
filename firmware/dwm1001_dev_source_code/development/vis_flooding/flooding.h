#include "nrf_drv_rng.h"
#include "v_state.h"

#define OTP_ANT_DLY 0x01C

void rx_ok_cb(const dwt_cb_data_t *cb_data);

void rx_to_cb(const dwt_cb_data_t *cb_data);

void rx_err_cb(const dwt_cb_data_t *cb_data);

void tx_conf_cb(const dwt_cb_data_t *cb_data);

static uint16 getAntDly(uint8 prf);

void transmitData(uint8 *data, uint16 length);
static uint64 getRandNum(void);