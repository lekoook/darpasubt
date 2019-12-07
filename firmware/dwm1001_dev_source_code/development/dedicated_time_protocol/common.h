/* Macros definitions */
#define NODES 4 /**< Number of nodes */
// Print enables (set to 1 to print)
#define P_DIST 1
#define P_TEMP 1
// Range measurement mode
#define RNG_MODE 0 // Set 1 to use measurement mode. Send 'r' through UART to start/reset.
// Ranging related
#define NODE_ID 0 // Node ID
#define RANGE_FREQ 10 // Frequency of the cycles
#define TX_INTERVAL 4000 // In microseconds
#define WAKE_BUFFER 1000 // Buffer time in microseconds to allow proper transceiver hardware waking.
#define UUS_TO_DWT_TIME 63897
#define RX_TX_BUFFER 3600 // Buffer time (us) to transit from RX to TX.

/*
  Working Configurations (NODES = 4)
  
  2 active devices (0, 1):
  TX_INTERVAL = 2000
  RX_TX_BUFFER = 1800
  RANGE_FREQ = 1

  3 active devices (0, 1, 2):
  TX_INTERVAL = 2000
  RX_TX_BUFFER = 1800
  RANGE_FREQ = 1

  4 active devices (0, 1, 2, 3):
  TX_INTERVAL = 2900
  RX_TX_BUFFER = 1800
  RANGE_FREQ = 40


*/

typedef unsigned long long uint64;