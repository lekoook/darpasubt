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
#include "uart.h"
#include "flooding.h"
#include "nrf_drv_gpiote.h"

//-----------------dw1000----------------------------

/*DW1000 config function*/
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

/* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
#define PRE_TIMEOUT 1000

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100 

//--------------dw1000---end---------------


#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      2000          /**< Timer period. LED1 timer will expire after 1000 ms */

#ifdef USE_FREERTOS

TaskHandle_t  ss_initiator_task_handle;   /**< Reference to SS TWR Initiator FreeRTOS task. */
extern void ss_initiator_task_function (void * pvParameter);
TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
#endif

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
    /* Delay a task for a given number of ticks */
    vTaskDelay(TASK_DELAY);
    /* Tasks must be implemented to never return... */
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
#else

  extern int flood(void);

#endif   // #ifdef USE_FREERTOS

// Global variables
uint16 antDelay;


int main(void)
{
  /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );

  #ifdef USE_FREERTOS
    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    /* Create task for SS TWR Initiator set to 2 */
    UNUSED_VARIABLE(xTaskCreate(ss_initiator_task_function, "SSTWR_INIT", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_initiator_task_handle));
  #endif // #ifdef USE_FREERTOS
  
  //-------------dw1000  ini------------------------------------	

  /* Setup NRF52832 interrupt on GPIO 25 : connected to DW1000 IRQ*/
  vInterruptInit();
	
  /*Initialization UART*/
  boUART_Init ();
  printf("Visibility Flooding Application \r\n");
	
  /* Reset DW1000 */
  reset_DW1000(); 

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();			
  
  /* Init the DW1000 */
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

  // Set SPI to 8MHz clock  
  port_set_dw1000_fastrate();

  /* Configure DW1000. */
  dwt_configure(&config);

  /* Initialization of the DW1000 interrupt*/
  /* Callback are defined in ds_twr_init_main.c */
  dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

  /* Set preamble timeout for expected frames. See NOTE 3 below. */
  //dwt_setpreambledetecttimeout(0); // PRE_TIMEOUT
          
   // Disable receiver timeout.
  dwt_setrxtimeout(0);

  dwt_setrxaftertxdelay(100);
  
  // We can turn on the receiver immediately.
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Initialise the Random Number Generator module.
  nrf_drv_rng_init(NULL);

  //-------------dw1000  ini------end---------------------------	
  // IF WE GET HERE THEN THE LEDS WILL BLINK
  struct Node node;
  Node_constructor(&node, 2, 8);
  node.v_states[node.id].degree = 3;
  while (1)
  {
    int i = 0;
    printf("id: %u\r\n", node.id);
    for (i = 0; i < 8; i++)
    {
      printf("i = %d\r\n", i);
      printf("degree: %u\r\n", node.v_states[i].degree);
      printf("is_new: %u\r\n", node.v_states[i].is_new);
      printf("state : ");
      int j;
      for (j = 0; j < 16; j++)
      {
        printf("%u", (node.v_states[i].state >> j) & 0x1);
      }
      printf("\r\n");
    }
    deca_sleep(1000);
    uint8 buf[3 * 8 + 1] = {0};
    uint16 len;
    len = Node_pack(&node, buf);
    printf("id = %d\r\n", buf[0]);
    for (i = 1; i < len; i+=3)
    {
      printf("bits: ");
      int j;
      for (j = 7; j >= 0; j--)
      {
        printf("%u", (buf[i] >> j) & 0x1);
      }
      for (j = 7; j >= 0; j--)
      {
        printf("%u", (buf[i+1] >> j) & 0x1);
      }
      for (j = 7; j >= 0; j--)
      {
        printf("%u", (buf[i+2] >> j) & 0x1);
      }
      printf("\r\n");
    }
    deca_sleep(1000);
    Node_unpack(&node, buf, 8);
    printf("id: %u\r\n", node.id);
    for (i = 0; i < 8; i++)
    {
      printf("i = %d\r\n", i);
      printf("degree: %u\r\n", node.v_states[i].degree);
      printf("is_new: %u\r\n", node.v_states[i].is_new);
      printf("state : ");
      int j;
      for (j = 0; j < 16; j++)
      {
        printf("%u", (node.v_states[i].state >> j) & 0x1);
      }
      printf("\r\n");
    }
    deca_sleep(3000);
    // flood();
  }
}

/*DWM1000 interrupt initialization and handler definition*/

/*!
* Interrupt handler calls the DW1000 ISR API. Call back corresponding to each event defined in ss_init_main
*/
void vInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  dwt_isr(); // DW1000 interrupt service routine 
}

/*!
* @brief Configure an IO pin as a positive edge triggered interrupt source.
*/
void vInterruptInit (void)
{
  ret_code_t err_code;

  if (nrf_drv_gpiote_is_init())
    printf("nrf_drv_gpiote_init already installed\n");
  else
    nrf_drv_gpiote_init();

  // input pin, +ve edge interrupt, no pull-up
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

  // Link this pin interrupt source to its interrupt handler
  err_code = nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, vInterruptHandler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);
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
  uint16 antDelay = 0;
  dwt_otpread(OTP_ANT_DLY, &rawAntDelay, 1); // '1' Refers to one 32 bits word.
  if (prf == DWT_PRF_16M)
  {
    antDelay = rawAntDelay & 0x0000FFFF; // PRF 16M Antenna delay is found in the lower 16 bits.
  }
  else if (prf == DWT_PRF_64M)
  {
    antDelay = rawAntDelay >> 16;
  }
  return antDelay;
}
