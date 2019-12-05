/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "flooding.h"

/* Frames used in the ranging process. See NOTE 1,2 below. */
static uint8 msg_in_frame[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 data[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define CMP_LEN 10
#define MSG_LEN 40
#define PAYLOAD_LEN MSG_LEN - CMP_LEN
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define CRC_LEN 2

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 50
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter 


/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/
int flood(void)
{
  transmitData(data, sizeof(data));
  // Generate a random time to wait.
  uint64 time = getRandNum() % 2000;
  deca_sleep(time);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_ok_cb()
*
* @brief Callback to process RX good frame events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  uint32 frame_len;

  /* A frame has been received, read it into the local buffer. */
  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
  if (frame_len <= RX_BUF_LEN)
  {
    dwt_readrxdata(rx_buffer, frame_len, 0);
  }

  /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
  rx_buffer[ALL_MSG_SN_IDX] = 0;
  if (memcmp(rx_buffer, msg_in_frame, CMP_LEN) == 0)
  {	
    rx_count++;
    
    // Print content.
    printf("DATA: ");
    int i;
    for (i = 0; i < RX_BUF_LEN; i++)
    {
      if ((i % 4 == 0) && (i != 0))
      {
        printf(" ");
      }
      printf("%x", rx_buffer[i]);
    }
    printf("\r\n");
  }

  // Re-enable RX
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_to_cb()
*
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  /* Reset RX to properly reinitialise LDE operation. */
  dwt_rxreset();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  /* TESTING BREAKPOINT LOCATION #2 */
  printf("TimeOut\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_err_cb()
*
* @brief Callback to process RX error events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  /* Reset RX to properly reinitialise LDE operation. */
  dwt_rxreset();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  /* TESTING BREAKPOINT LOCATION #3 */
  printf("Transmission Error : may receive package from different UWB device\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn tx_conf_cb()
*
* @brief Callback to process TX confirmation events
*
* @param  cb_data  callback data
*
* @return  none
*/
void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  printf("TX\r\n");
}

void transmitData(uint8 *data, uint16 length)
{
  uint8 buf[PAYLOAD_LEN] = {0};
  
  dwt_forcetrxoff();

  // Prepare for transmission.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  dwt_writetxdata(length, data, 0);
  dwt_writetxfctrl(length, 0, 1);

  // Begin transmission.
  int res = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  tx_count++;
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