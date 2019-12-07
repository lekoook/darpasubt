/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "sdk_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_rng.h"
#include "nrf_uart.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "app_error.h"
#include <string.h>
#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "nrf_drv_gpiote.h"
#include "app_timer.h"
#include "UART.h"
#include "ranging.h"


//-----------------dw1000----------------------------

static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

//--------------dw1000---end---------------

/* Macros definitions */
// Ranging related
#ifdef USE_FREERTOS
TaskHandle_t run_task_handle;   /**< Reference to SS TWR Initiator FreeRTOS task. */
extern void runTask (void * pvParameter);
TaskHandle_t led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
#endif

void runTask (void * pvParameter);
static uint64 getRandNum(void);
static void initListen(void);
static uint16 getAntDly(uint8 prf);
static void ledOn(void);

/* Global variables */
APP_TIMER_DEF(activeInit);
APP_TIMER_DEF(wakeInit);
APP_TIMER_DEF(rx1Init); // First RX phase timer
APP_TIMER_DEF(rx2Init); // Second RX phase timer
APP_TIMER_DEF(active);
APP_TIMER_DEF(wake);
APP_TIMER_DEF(rx1); // First RX phase timer
APP_TIMER_DEF(rx2); // Second RX phase timer
APP_TIMER_DEF(cycle); // Repeat timer to time each cycle period.


#ifdef USE_FREERTOS

  /**@brief LED0 task entry function.
   *
   * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
   */
  static void led_toggle_task_function (void * pvParameter)
  {
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {
      LEDS_INVERT(BSP_LED_0_MASK);
      // Delay a task for a given number of ticks
      vTaskDelay(TASK_DELAY);
      // Tasks must be implemented to never return...
    }
  }

  /**@brief The function to call when the LED1 FreeRTOS timer expires.
   *
   * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
   */
  static void led_toggle_timer_callback (void * pvParameter)
  {
    UNUSED_PARAMETER(pvParameter);
    LEDS_INVERT(BSP_LED_1_MASK);
  }
#endif   // #ifdef USE_FREERTOS

int main(void)
{
  // Setup some LEDs for debug Green and Blue on DWM1001-DEV
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );

  #ifdef USE_FREERTOS
    // Create task for LED0 blinking with priority set to 2
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    // Start timer for LED1 blinking
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    // Create task for SS TWR Initiator set to 2
    UNUSED_VARIABLE(xTaskCreate(runTask, "entry task", configMINIMAL_STACK_SIZE + 200, NULL, 2, &run_task_handle));
  #endif // #ifdef USE_FREERTOS
  
  //-------------dw1000  ini------------------------------------	

  // Setup NRF52832 interrupt on GPIO 25 : connected to DW1000 IRQ
  vInterruptInit();
  
  // Initialization UART
  boUART_Init();
  
  // Reset DW1000
  reset_DW1000(); 

  // Set SPI clock to 2MHz
  port_set_dw1000_slowrate();			

  
  // Init the DW1000
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    //Init of DW1000 Failed
    while (1) {};
  }

  // We need to read antenna delay in OTP memory with slow SPI clock.
  // We set it and NEVER change it, as it will be used for all future timestamping purpose.
  antDelay = getAntDly(DWT_PRF_64M);
  dwt_settxantennadelay(antDelay);
  dwt_setrxantennadelay(antDelay);

  // Set SPI clock to 8MHz
  port_set_dw1000_fastrate();

  // Configure DW1000.
  dwt_configure(&config);

  // Initialization of the DW1000 transceiver interrupts
  // Callback are defined in int_handler module
  dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

  // Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors).
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

  // Prepare for ranging process.
  struct ranging_timers timers;
  timers.activeInitTimer = &activeInit;
  timers.wakeInitTimer = &wakeInit;
  timers.rx1InitTimer = &rx1Init;
  timers.rx2InitTimer = &rx2Init;
  timers.activeTimer = &active;
  timers.wakeTimer = &wake;
  timers.rx1Timer = &rx1;
  timers.rx2Timer = &rx2;
  timers.cycleTimer = &cycle;
  initRanging(timers);

  // Initialise the Random Number Generator module.
  nrf_drv_rng_init(NULL);
  if (NODE_ID == 0) nodeFunction = MASTER_FUNCTION;
  else nodeFunction = SLAVE_FUNCTION;
  ledOn();

  //-------------dw1000  ini------end---------------------------	
  // IF WE GET HERE THEN THE LEDS WILL BLINK

  #ifdef USE_FREERTOS		
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();	

    while(1) 
    {};
  #else
    int pvParameter = 0;
    runTask(&pvParameter);
  #endif
}

/**
 * @brief Node entry function.
 * 
 * @param[in] pvParameter Pointer that will be used as the parameter for the task.
 */
void runTask (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  dwt_setleds(DWT_LEDS_ENABLE);

  // Begin the tracking of ranging cycles immediately.
  if (nodeFunction == MASTER_FUNCTION)
  {
    masterRanging();
  }
  else
  {
    initListen();
  }
  
  
  // Application loop.
  while(true)
  {
    uint32 sys = dwt_read32bitreg(SYS_STATUS_ID);
    // Cycle no longer in sync with master node. Take remedial actions.
    if (hasSyncErr)
    {
      // Stop cycle timer and reset flags and transreceiver.
      lowTimerStopAll();
      dwt_forcetrxoff();
      dwt_rxreset();
      hasSyncErr = false;

      // Attempt to look for master node again.
      initListen();
    }
  }
}

/**
 * @brief Get a random number using the nRF Random Number Generator (RNG) module.
 * 
 * @return uint64 randomly generated 64 bit unsigned integer.
 */
static uint64 getRandNum(void)
{
  uint8 buf[8] = {0};
  uint8 bytesAvail = 0;
  uint64 ret = 0;
  int i;

  // Get the random generated bytes from the module.
  do
  {
    nrf_drv_rng_bytes_available(&bytesAvail);
  } while (bytesAvail < 8); // Wait until there are enough generated random bytes.
  
  // Read and return the random generated number.
  nrf_drv_rng_rand(buf, 8);
  for (i = 0; i < 8; i++)
  {
    ret |= buf[i];
    ret <<= 8;
  }

  return ret;
}
/**
 * @brief Listens for master node in the network. Once master node is found, this node syncs it's cycle with master.
 *        This is handled by rxHandler() function.
 * 
 * 
 */
static void initListen(void)
{
  // Wait for random amount of time before enabling RX.
  uint64 random = getRandNum();
  uint16 time = (random % 500) + 1000; // Get a range between 10000 - 15000.
  nrf_delay_ms(time); // Delay in microseconds.
  
  isInitiating = true;

  // Start listening for one cycle
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/**
 * @brief Retrieves the antenna delay stored in OTP memory.
 * 
 * @param prf the PRF used to configure DWM1000.
 * @return antenna delay from OTP memory.
*/
static uint16 getAntDly(uint8 prf)
{
  uint32 rawAntDelay;
  uint16 delay = 0;
  dwt_otpread(OTP_ANT_DLY, &rawAntDelay, 1); // '1' Refers to one 32 bits word.
  if (prf == DWT_PRF_16M)
  {
    delay = rawAntDelay & 0x0000FFFF; // PRF 16M Antenna delay is found in the lower 16 bits.
  }
  else if (prf == DWT_PRF_64M)
  {
    delay = rawAntDelay >> 16;
  }
  return delay;
}

/**
 * @brief Turn on master or slave status LED.
 * 
 */
static void ledOn(void)
{
  LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  if (nodeFunction == MASTER_FUNCTION) 
  {
    LEDS_ON(BSP_LED_2_MASK);
  }
  else
  {
    LEDS_ON(BSP_LED_0_MASK);
  }
}