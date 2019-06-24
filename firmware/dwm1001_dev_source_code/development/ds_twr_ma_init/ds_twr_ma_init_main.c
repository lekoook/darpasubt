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
#include "ds_twr_ma_init_main.h"
#include "timestamper.h"

#define APP_NAME "DS TWR MA INITIATOR"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 250

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 ?s and 1 ?s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536
// Time between initiator's first TX and second TX. In microseconds.
#define FIRST_TO_SECOND_TX_UUS  43700

/* Frames used in the ranging process. See NOTE 1,2 below. */
static uint8 req_rng_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 data_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'A', 'T', 'A', 0xE0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 ack_rng_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'E', 'Q', 'R', 0xE0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define INIT_MSG_TYPE_IDX 10
#define RESP_MSG_TYPE_IDX 10
#define DATA_MSG_LEN 38
#define DATA_MSG_TS_LEN 5
#define DATA_MSG_TX_1_IDX 11
#define DATA_MSG_TX_2_IDX 16
#define DATA_MSG_RX_1_IDX 21
// Frames related
#define MSG_TYPE_RNG_REQ 0
#define MSG_TYPE_DATA 1
#define MSG_TYPE_ACK 2
#define ACK_EXP_COUNT 3 // Expected number of acknowledgements for each cycle.
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 38
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static void resetFlags(void);
static void writeDataTs(uint8 *field, const uint64 ts, uint16 len);

/*Interrupt flag*/
static volatile int tx_int_flag = 0; // Transmit success interrupt flag
static volatile int to_int_flag = 0; // Timeout interrupt flag
static volatile int er_int_flag = 0; // Error interrupt flag 
static volatile int ack_rng_flag = 0; // Receive acknowledgement success flag
static volatile int data_flag = 0; // Receive data flag
static volatile int wait_ack_flag = 0; // Wait ACK flag

/*Transactions Counters */
static volatile uint8 req_count = 0 ; // Successful transmit counter
static volatile uint8 rng_ack_count = 0 ; // Successful acknowledgement counter 
static volatile uint8 data_count = 0 ; // Successful data message counter

/* TX and RX timestamps for distance calculation. */
uint64 initTx1 = 0;
uint64 initTx2 = 0;
uint64 initAckRx[ACK_EXP_COUNT + 5] = {0}; // Plus 5 as buffer.
uint64 dataMsgDly = 0;
uint64 dataMsgTs = 0;
uint32 dataTxDly = 0;
extern uint16 antDelay;

/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/
int ds_twr_ma_run(void)
{
  #ifdef USE_DS_TWR_MA

  // Begin each range as long as we are not waiting for acknowledgements.
  if (!wait_ack_flag)
  {
    // Increase range request counter and write to frame.
    req_count++;
    req_rng_msg[ALL_MSG_SN_IDX] = req_count;
    // Write transmission frame data and prepare transmission.
    dwt_writetxdata(sizeof(req_rng_msg), req_rng_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(req_rng_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    // Start transmission.
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /*Waiting for transmission success flag*/
    while (!(tx_int_flag)) {};

    // printf("(I) Range Request # : %d\r\n", req_count);

    // Read the TX timestamp.
    initTx1 = getTxTimestampU64();

    /*Reseting tx interrupt flag*/
    tx_int_flag = 0;
  }

  /* Now We wait for acknowledgements from responder. */
  while(!(ack_rng_flag || to_int_flag|| er_int_flag)) {};

  if (er_int_flag)
  {
    er_int_flag = 0;
    printf("RX error\r\n");
    dwt_rxreset();
    // Reset all the flags for next range cycle.
    resetFlags();
    return 0;
  }

  if (to_int_flag)
  {
    to_int_flag = 0;
    printf("Timeout\r\n");
    dwt_rxreset();
    // Reset all the flags for next range cycle.
    resetFlags();
    return 0;
  }

  if (ack_rng_flag)
  {
    /* Check that the frame is the expected range request from the companion "DS TWR MA Responder" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rng_ack_count = rx_buffer[ALL_MSG_SN_IDX]; // We read the ack count first before clearing.
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, ack_rng_msg, ALL_MSG_COMMON_LEN) == 0)
    {
      printf("(I) Ack # : %d\r\n", rng_ack_count);
    }

    // Read the RX timestamp for each successful acknowledgement received.
    initAckRx[rng_ack_count - 1] = getRxTimestampU64();

    // Resetting range acknowledgement flag.
    ack_rng_flag = 0;

    if (rng_ack_count == ACK_EXP_COUNT)
    {
      data_flag = 1;
      wait_ack_flag = 0;
    }
    else
    {
      // Re-enable RX for next ACK.
      dwt_rxenable(DWT_START_RX_IMMEDIATE);

      wait_ack_flag = 1;
    }
  }

  if (data_flag)
  {
    /* First we will write all the timestamps required by responder to data message. */
    // Calculate the delayed TX timestamp for data message TX.
    uint64 txInterval = FIRST_TO_SECOND_TX_UUS;
    uint64 conv = UUS_TO_DWT_TIME;
    dataMsgDly = (initTx1 + (txInterval * conv));
    dataTxDly = dataMsgDly >> 8;
    dataMsgTs = dataMsgDly + antDelay;
    writeDataTs(&data_msg[DATA_MSG_TX_1_IDX], initTx1, DATA_MSG_TS_LEN);
    writeDataTs(&data_msg[DATA_MSG_TX_2_IDX], dataMsgTs, DATA_MSG_TS_LEN);
    int i;
    for (i = 0; i < ACK_EXP_COUNT; i++)
    {
      writeDataTs(&data_msg[DATA_MSG_RX_1_IDX + (i * DATA_MSG_TS_LEN)], initAckRx[i], DATA_MSG_TS_LEN);
    }
    
    /* Now we send data to the responder. */
    // Increase data count and write to frame.
    data_count++;
    data_msg[ALL_MSG_SN_IDX] = data_count;
    // Write transmission frame data and prepare transmission.
    dwt_writetxdata(sizeof(data_msg), data_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(data_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    
    dwt_setdelayedtrxtime(dataTxDly);

    // Start transmission.
    dwt_starttx(DWT_START_TX_DELAYED);

    /*Waiting for transmission success flag*/
    while (!(tx_int_flag)) {};

    printf("(I) Data Outgoing # : %d\r\n", data_count);

    // Resetting the data flag.
    data_flag = 0;

    // Resetting TX interrupt flag.
    tx_int_flag = 0;

    deca_sleep(30);
  }


  #endif

  /* Loop forever initiating ranging exchanges. */

  #ifdef USE_SS_TWR
  /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
  req_rng_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_writetxdata(sizeof(req_rng_msg), req_rng_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(req_rng_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);


  /*Waiting for transmission success flag*/
  while (!(ack_rng_flag))
  {};

  if (ack_rng_flag)
  {
    req_count++;
    printf("Transmission # : %d\r\n",req_count);

    /*Reseting tx interrupt flag*/
    ack_rng_flag = 0 ;
  }

  /* Wait for reception, timeout or error interrupt flag*/
  while (!(ack_rng_flag || to_int_flag|| er_int_flag))
  {};

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (ack_rng_flag)
  {		
    uint32 frame_len;

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the companion "SS TWR responder" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
    {	
      ack_count++;
      printf("Reception # : %d\r\n",ack_count);
      float reception_rate = (float) ack_count / (float) req_count * 100;
      printf("Reception rate # : %f\r\n",reception_rate);
      uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
      int32 rtd_init, rtd_resp;
      float clockOffsetRatio ;

      /* Retrieve poll transmission and response reception timestamps. See NOTE 4 below. */
      poll_tx_ts = dwt_readtxtimestamplo32();
      resp_rx_ts = dwt_readrxtimestamplo32();

      /* Read carrier integrator value and calculate clock offset ratio. See NOTE 6 below. */
      clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

      /* Get timestamps embedded in response message. */
      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
      resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

      /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
      rtd_init = resp_rx_ts - poll_tx_ts;
      rtd_resp = resp_tx_ts - poll_rx_ts;

      tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
      distance = tof * SPEED_OF_LIGHT;
      printf("Distance : %f\r\n",distance);

      /*Reseting receive interrupt flag*/
      ack_rng_flag = 0; 
    }
   }

  if (to_int_flag || er_int_flag)
  {
    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();

    /*Reseting interrupt flag*/
    to_int_flag = 0 ;
    er_int_flag = 0 ;
  }

    /* Execute a delay between ranging exchanges. */
    //     deca_sleep(RNG_DELAY_MS);
    //	return(1);

  #endif
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
  /* A frame has been received, read it into the local buffer. */
  uint32 frame_len;
  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
  if (frame_len <= RX_BUF_LEN)
  {
    dwt_readrxdata(rx_buffer, frame_len, 0);

    if (rx_buffer[RESP_MSG_TYPE_IDX] == MSG_TYPE_ACK)
    {
      ack_rng_flag = 1;
    }
  }
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
  to_int_flag = 1 ;
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
  er_int_flag = 1 ;
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
  /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
  * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
  * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
  * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
  * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

  tx_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #4 */
}

/**
 * @brief Resets all range state flags to their default value.
 * 
 */
static void resetFlags(void)
{
  wait_ack_flag = 0;
  data_flag = 0;
  ack_rng_flag = 0;
  tx_int_flag = 0;
  to_int_flag = 0;
  er_int_flag = 0;
}

/**
 * @brief Writes a timestamp into the data message frame field that will be transmitted.
 * 
 * @param field pointer to the byte in the field to begin filling from.
 * @param ts timestamp value to fill up with.
 * @param len length of the timestamp in bytes.
 */
static void writeDataTs(uint8 *field, const uint64 ts, uint16 len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    field[i] = (ts >> (i * 8)) & 0xFF;
  }
}

/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);

  while (true)
  {
    ss_init_run();
    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}
/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 6. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
