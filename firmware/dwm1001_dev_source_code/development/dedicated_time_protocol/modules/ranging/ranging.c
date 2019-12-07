#include "ranging.h"

const app_timer_id_t *activeInitTimer = NULL;
const app_timer_id_t *wakeInitTimer = NULL;
const app_timer_id_t *rx1InitTimer = NULL;
const app_timer_id_t *rx2InitTimer = NULL;
const app_timer_id_t *activeTimer = NULL;
const app_timer_id_t *wakeTimer = NULL;
const app_timer_id_t *rx1Timer = NULL;
const app_timer_id_t *rx2Timer = NULL;
const app_timer_id_t *cycleTimer = NULL;
uint32 cyclePeriod = 0;
uint32 activePeriod = 0;
uint32 sleepPeriod = 0;
uint32 wakePeriod = 0;
bool isInitiating = false;
bool tx1Sending = false;
bool tx2Sending = false;
bool hasSyncErr = false;
enum NodeFunction nodeFunction = MASTER_FUNCTION;
msg_template txMsg1;
msg_template txMsg2;
TxStatus txStatus = TX_SUCCESS;
uint8 masterId = 0;
uint8 syncErrCount = 0;
uint8 header[HEADER_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0};
uint64 tsTable[NUM_STAMPS_PER_CYCLE][NODES] = {{0}};
uint32 rxTimeout1 = 0;
uint32 rxTimeout2 = 0;
uint16 antDelay = 0;
uint64 varDelay = 0;
uint64 regDelay = 0;


/*************************************************************************************************/
/*********************************** TIMER HANDLER FUNCTIONS *************************************/
/*************************************************************************************************/
/**
 * @brief Handler function for when the sleeping phase timeouts and is over.
 * 
 * @param pContext General parameter that can be passed into the handler.
 */
static void wakeTimerHandler(void *pContext)
{
  // printf("wake\r\n");
  // dwWake();
  if (nodeFunction == SLAVE_FUNCTION)
  {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
}

/**
 * @brief Handler function for when active period timer timeouts.
 * 
 * @param pContext General parameter that can be passed into the handler.
 */
static void activeTimerHandler(void *pContext)
{
  // printf("active\r\n");
  goToSleep(true, sleepPeriod, wakePeriod);

  // printTable(tsTable);
  printOutput(tsTable, NODE_ID);
  initTable(tsTable);
}

/**
 * @brief Handler function for when the first phase of RX is over.
 * 
 * @param pContext General parameter that can be passed into the handler.
 */
static void rx1TimeoutHandler(void *pContext)
{
  // printf("rx1\r\n");
  tx1Sending = true;
  txStatus = convertTx(&txMsg1, (DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED));
  if (txStatus == TX_SUCCESS)
  {
    // Do nothing.
  }
  else
  {
    tx1Sending = false;
  }
}

/**
 * @brief Handler function for when the second phase of RX is over.
 * 
 * @param pContext General parameter that can be passed in the handler.
 */
static void rx2TimeoutHandler(void *pContext)
{
  // printf("rx2\r\n");
  uint64 refTs = tsTable[IDX_TS_1][NODE_ID]; // First TX timestamp of this Node.
  uint64 tx2Ts = refTs + regDelay + (uint64)antDelay;
  
  updateTable(tsTable, txMsg2, tx2Ts, NODE_ID);
  writeTx2(&txMsg2, tsTable);
  tx2Sending = configTx(tsTable, regDelay, refTs, &txMsg2);
}

/**
 * @brief Handler function for tracking the start/end of each ranging cycle.
 * 
 * @param pContext General parameter that can be passed in the handler.
 */
static void cycleHandler(void *pContext)
{
  if (nodeFunction == MASTER_FUNCTION)
  {
    tx1Sending = true;

    txStatus = convertTx(&txMsg1, (DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED));
    if (txStatus == TX_SUCCESS)
    {
    }
    else
    {
      tx1Sending = false;
    }
  }
  else
  {
    // Check for range value errors with master node.
    if (syncErrCount >= 10)
    {
      hasSyncErr = true;
      syncErrCount = 0;
    }
    else
    {
    }
  }
}

/**
 * @brief Handler function for when the active phase is about to begin.
 * 
 * @param pContext General parameter that can be passed in the handler.
 */
static void activeInitHandler(void *pContext)
{
  lowTimerStart(*activeTimer, cyclePeriod);
}

/**
 * @brief Handler function for when the first phase of RX is about to begin.
 * 
 * @param pContext General parameter that can be passed in the handler.
 */
static void rx1InitHandler(void *pContext)
{
  lowTimerStart(*rx1Timer, cyclePeriod);
}

/**
 * @brief Handler function for when the second phase of RX is about to begin.
 * 
 * @param pContext General parameter that can be passed in the handler.
 */
static void rx2InitHandler(void *pContext)
{
  lowTimerStart(*rx2Timer, cyclePeriod);
}

/**
 * @brief Handler function for when the sleeping phase is about to begin.
 * 
 * @param pContext General parameter that can be passed in the handler.
 */
static void wakeInitHandler(void *pContext)
{
  lowTimerStart(*wakeTimer, cyclePeriod);
}
/*************************************************************************************************/
/********************************* TIMER HANDLER FUNCTIONS END ***********************************/
/*************************************************************************************************/


/**
 *@brief Prints a 64bit unsigned timestamp value.
 *
 *@details This is for debugging purpose.
 * 
 *@param val value to be printed.
 */
void printTs(uint64 val)
{
  int i;
  uint8 ts;

  for (i = 4; i >= 0; i--)
  {
    ts = (uint8)(val >> (i * 8));
    printf("%.2x", ts);
  }
}

/**
 * @brief Prints all required output data.
 * 
 * @param table pointer to the 2D array representing the timestamp table.
 * @param thisId identifier of the calling node.
 */
static void printOutput(uint64 table[NUM_STAMPS_PER_CYCLE][NODES], uint8 thisId)
{
  // Calculate the distances for each of other nodes.
  double dists[NODES] = {0};
  for (int i = 0; i < NODES; i++)
  {
    if (i == thisId)
    {
      continue;
    }

    dists[i] = calcDist(table, i);

    // Track the consecutive occurences of value errors with master node.
    if (dists[i] == -1 && i == 0)
    {
      syncErrCount++;
    }
    else if (dists[i] != -1 && i == 0)
    {
      syncErrCount = 0;
    }
  }

  // Retrieve temperature from register.
  uint16 value = dwt_readtempvbat(1); // Pass in '1' for SPI > 3MHz
  value = value >> 8; // Temperature is at the higher 8 bits.
  double temp = 1.13 * value - 113.0; // Formula to get real temperature in Celsius. See user manual.

  printData(dists, temp, thisId);
}

/**
 *@brief Prints the entire timestamps table.
 *
 *@details This is for debugging purpose.
 *
 *@param table 2D containing all the timestamps.
 */
static void printTable(uint64 table[NUM_STAMPS_PER_CYCLE][NODES])
{
  int i,j;
  
  printf("\r\n\n");
  for (i = 0; i < NUM_STAMPS_PER_CYCLE; i++)
  {
    for (j = 0; j < NODES; j++)
    {
      printf("|");
      printTs(table[i][j]);
      printf("|\t");
    }
    printf("\r\n");
  }
  printf("\r\n");
}

/**
 * @brief Initialises all variables required for ranging process.
 * 
 */
void initRanging(struct ranging_timers timers)
{
  // Initialise all timers.
  activeInitTimer = timers.activeInitTimer;
  wakeInitTimer = timers.wakeInitTimer;
  rx1InitTimer = timers.rx1InitTimer;
  rx2InitTimer = timers.rx2InitTimer;
  activeTimer = timers.activeTimer;
  wakeTimer = timers.wakeTimer;
  rx1Timer = timers.rx1Timer;
  rx2Timer = timers.rx2Timer;
  cycleTimer = timers.cycleTimer;
  lowTimerInit();
  lowTimerSingleCreate(activeInitTimer, activeInitHandler);
  lowTimerSingleCreate(wakeInitTimer, wakeInitHandler);
  lowTimerSingleCreate(rx1InitTimer, rx1InitHandler);
  lowTimerSingleCreate(rx2InitTimer, rx2InitHandler);
  lowTimerRepeatCreate(activeTimer, activeTimerHandler);
  lowTimerRepeatCreate(wakeTimer, wakeTimerHandler);
  lowTimerRepeatCreate(rx1Timer, rx1TimeoutHandler);
  lowTimerRepeatCreate(rx2Timer, rx2TimeoutHandler);
  lowTimerRepeatCreate(cycleTimer, cycleHandler);

  // Pre-calculate all the timings in one cycle (ie, cycle, active, sleep period).
  initCycleTimings();

  // Initialise timestamp-related data structures.
  initTable(tsTable);

  // Initalises the messages to be transmitted.
  initTxMsgs(&txMsg1, &txMsg2);

  // Prints configuration information.
  printConfig(
    NODES,
    NODE_ID,
    RANGE_FREQ,
    TX_INTERVAL,
    RX_TX_BUFFER,
    cyclePeriod,
    activePeriod,
    sleepPeriod,
    WAKE_BUFFER
  );
}

/**
 * @brief Start all timers for a master node.
 * 
 */
void masterRanging(void)
{
  lowTimerStart(*cycleTimer, cyclePeriod);
  lowTimerStart(*rx2InitTimer, rxTimeout2);
  lowTimerStart(*activeInitTimer, activePeriod); // Begin to track active transceiving.
  lowTimerStart(*wakeInitTimer, wakePeriod);
}

/**
 * @brief Start all timers for a slave node.
 * 
 */
void slaveRanging(void)
{
  lowTimerStart(*cycleTimer, cyclePeriod);
  lowTimerStart(*rx1InitTimer, rxTimeout1);
  lowTimerStart(*rx2InitTimer, rxTimeout2);
  lowTimerStart(*activeInitTimer, activePeriod);
  lowTimerStart(*wakeInitTimer, wakePeriod);
}

/**
 * @brief Updates first TX timestamp in timestamp table and start timer for second TX.
 * 
 * @param ts timestamp of first TX.
 */
void txUpdate(uint64 ts)
{
  updateTable(tsTable, txMsg1, ts, NODE_ID);
}

/**
 * @brief Updates the timestamps table with reception timestamp and begin next phase timers.
 * 
 * @param msg msg_template struct used to determine the location to store the timestamp.
 */
void rxHandler(msg_template *msg)
{
  if (isInitiating)
  {
    isInitiating = false;
    if (msg->id == masterId && msg->isFirst == 1 && nodeFunction == SLAVE_FUNCTION)
    {
      slaveRanging();
    }
    return;
  }

  uint64 ts = getRxTimestampU64();
  updateTable(tsTable, *msg, ts, NODE_ID);
}

/**
 * @brief Puts DW1000 to sleep.
 * 
 * @details Note: \p sleep must be lesser than \p wake. Time is needed for the
 *          transceivers to initialise before actual reception can take place.
 *          Hence, there must be adequate time allocated for the transceiver to
 *          be ready before the next cycle begins.
 * 
 * @param rxOn rxOn set to true if the device needs to wake up with receiver on.
 * @param sleep time duration for actual hardware sleeping
 * @param wake time duration to the next cycle
 */
static void goToSleep(bool rxOn, uint32 sleep, uint32 wake)
{
  // dwSleep(rxOn); // Commented out for development purpose.
}

/**
 * @brief Calculates the distance using the timestamp table.
 * 
 * @param table pointer to the 2D array representing the timestamp table.
 * @param id identifier of the node to calculate the distance with.
 * @return double the calculated distance.
 */
static double calcDist(uint64 table[NUM_STAMPS_PER_CYCLE][NODES], uint8 id)
{
  double tof, roundTrip1, roundTrip2, replyTrip2, replyTrip1;
  uint64 ts[NUM_STAMPS_PER_CYCLE] = {0};

  // Get values to calculate.
  getFullTs(table, ts, NODE_ID, id);

  int i;
  for (i = 0; i < NUM_STAMPS_PER_CYCLE; i++)
  {
    if (ts[i] == 0)
    {
      return -1;
    }
  }

  roundTrip1 = (double)((ts[IDX_TS_4] - ts[IDX_TS_1]) * DWT_TIME_UNITS);
  roundTrip2 = (double)((ts[IDX_TS_6] - ts[IDX_TS_3]) * DWT_TIME_UNITS);
  replyTrip1 = (double)((ts[IDX_TS_3] - ts[IDX_TS_2]) * DWT_TIME_UNITS);
  replyTrip2 = (double)((ts[IDX_TS_5] - ts[IDX_TS_4]) * DWT_TIME_UNITS);

  tof = (roundTrip1 * roundTrip2 - replyTrip1 * replyTrip2) / (roundTrip1 + roundTrip2 + replyTrip1 + replyTrip2);

  return tof * SPEED_OF_LIGHT;
}

/**
 * @brief Initialises the cycle timings.
 * 
 * @details These timings are the cycle, active and sleep periods.
 */
static void initCycleTimings(void)
{
  uint64 interval;

  cyclePeriod = 1000000 / RANGE_FREQ; // Convert from seconds to microseconds.
  activePeriod = (2 * NODES * TX_INTERVAL); // Number of intervals in NODES nodes.
  sleepPeriod = cyclePeriod - activePeriod;
  wakePeriod = cyclePeriod - WAKE_BUFFER;
  
  // RX timeout values for synchronising TX moments.
  rxTimeout1 = ((uint32)TX_INTERVAL * (uint32)NODE_ID);
  rxTimeout2 = ((uint32)TX_INTERVAL * (uint32)NODES) + rxTimeout1;
  // The values are deducted with a fixed value to allow transition time from RX to TX for transceiver.
  rxTimeout1 -= (uint32)RX_TX_BUFFER;
  rxTimeout2 -= (uint32)RX_TX_BUFFER;
  
  interval = (uint64)TX_INTERVAL * (uint64)UUS_TO_DWT_TIME; // interval in DWT time units
  uint64 temp = 500; // TODO: We need this to allow TX2 to be configured early enough. However, this will push back the TX2 and potentially mess up with the next node attempting to receive before switching from RX to TX. Investigate.
  regDelay = (NODES * interval) + (temp * (uint64)UUS_TO_DWT_TIME);
  varDelay = (NODE_ID * interval);
}

/**
 * @brief Initialises the TX messages to be used during the cycle.
 * 
 * @param tx1 the first TX message.
 * @param tx2 the second TX message.
 */
static void initTxMsgs(msg_template *tx1, msg_template *tx2)
{
  tx1->id = NODE_ID;
  tx1->isFirst = 1;
  memcpy(tx1->header, header, HEADER_LEN);
  memset(tx1->data, 0, DATA_LEN);
  memset(tx1->crc, 0, CRC_LEN);

  tx2->id = NODE_ID;
  tx2->isFirst = 0;
  memcpy(tx2->header, header, HEADER_LEN);
  memset(tx2->data, 0, DATA_LEN);
  memset(tx2->crc, 0, CRC_LEN);
}
