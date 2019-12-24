#ifndef MESSAGE_TEMPLATE_H
#define MESSAGE_TEMPLATE_H
#include "message_template.h"
#endif

#ifndef MESSAGE_TRANSCEIVER_H
#define MESSAGE_TRANSCEIVER_H
#include "message_transceiver.h"
#endif

#ifndef INT_HANDLER_H
#define INT_HANDLER_H
#include "int_handler.h"
#endif

#include "low_timer.h"
#include "timestamper.h"
#include "printer.h"
#include "common.h"

#define SPEED_OF_LIGHT 299702547
#define OTP_ANT_DLY 0x01C

enum NodeFunction
{
  MASTER_FUNCTION,
  SLAVE_FUNCTION
};

// Timers related
extern const app_timer_id_t *activeInitTimer;
extern const app_timer_id_t *wakeInitTimer;
extern const app_timer_id_t *rx1InitTimer;
extern const app_timer_id_t *rx2InitTimer;
extern const app_timer_id_t *activeTimer;
extern const app_timer_id_t *wakeTimer;
extern const app_timer_id_t *rx1Timer;
extern const app_timer_id_t *rx2Timer;
extern const app_timer_id_t *cycleTimer;
extern uint32 cyclePeriod;
extern uint32 activePeriod;
extern uint32 sleepPeriod; // Time duration for actual hardware sleeping
extern uint32 wakePeriod; // Time duration from the last TX/RX to the start of next cycle

// States related
extern bool isInitiating;
extern bool tx1Sending;
extern bool tx2Sending;
extern bool hasSyncErr;
extern enum NodeFunction nodeFunction;

// Range variables
// msg_template is the entire frame to tx out. there is a frame format 
// (first 10 bytes and last 2 bytes) to follow, check the dw1000 manual.
extern msg_template txMsg1;
extern msg_template txMsg2;
extern TxStatus txStatus;
extern uint8 masterId;
extern uint8 syncErrCount;
extern uint8 header[HEADER_LEN];
extern uint64 tsTable[NUM_STAMPS_PER_CYCLE][NODES];
extern uint32 rxTimeout1;
extern uint32 rxTimeout2;

// Delay values
extern uint16 antDelay; // Once set, must NEVER be changed.
extern uint64 varDelay;
extern uint64 regDelay;

struct ranging_timers
{
  const app_timer_id_t *activeInitTimer;
  const app_timer_id_t *wakeInitTimer;
  const app_timer_id_t *rx1InitTimer;
  const app_timer_id_t *rx2InitTimer;
  const app_timer_id_t *activeTimer;
  const app_timer_id_t *wakeTimer;
  const app_timer_id_t *rx1Timer;
  const app_timer_id_t *rx2Timer;
  const app_timer_id_t *cycleTimer;
};

// Timer function prototypes
static void wakeTimerHandler(void *pContext);
static void activeTimerHandler(void *pContext);
static void rx1TimeoutHandler(void *pContext);
static void rx2TimeoutHandler(void *pContext);
static void cycleHandler(void *pContext);
static void activeInitHandler(void *pContext);
static void rx1InitHandler(void *pContext);
static void rx2InitHandler(void *pContext);
static void wakeInitHandler(void *pContext);

// Print function prototypes
void printTs(uint64 val);
static void printOutput(uint64 table[NUM_STAMPS_PER_CYCLE][NODES], uint8 thisId);
static void printTable(uint64 table[NUM_STAMPS_PER_CYCLE][NODES]);

void initRanging(struct ranging_timers timers);
void masterRanging(void);
void slaveRanging(void);
void txUpdate(uint64 ts);
void rxHandler(msg_template *msg);
static void goToSleep(bool rxOn, uint32 sleep, uint32 wake);
static double calcDist(uint64 table[NUM_STAMPS_PER_CYCLE][NODES], uint8 id);
static void initCycleTimings(void);
static void initTxMsgs(msg_template *tx1, msg_template *tx2);

