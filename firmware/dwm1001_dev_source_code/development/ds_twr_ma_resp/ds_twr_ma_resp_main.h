#define OTP_ANT_DLY 0x01C
// #define USE_SS_TWR // Uncomment to use SS_TWR technique.
#define USE_DS_TWR_MA

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint64 *ts);

int ss_init_run(void);

void rx_ok_cb(const dwt_cb_data_t *cb_data);

void rx_to_cb(const dwt_cb_data_t *cb_data);

void rx_err_cb(const dwt_cb_data_t *cb_data);

void tx_conf_cb(const dwt_cb_data_t *cb_data);

void ss_initiator_task_function (void * pvParameter);

static uint16 getAntDly(uint8 prf);