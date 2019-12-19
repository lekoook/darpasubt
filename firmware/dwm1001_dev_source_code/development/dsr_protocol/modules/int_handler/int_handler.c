#include "int_handler.h"

/* Local functions prototypes */
void vInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

/* Public functions */
/**
 * @brief Configure an IO pin as a positive edge triggered interrupt source.
 * 
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
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  handle_rx();
}

/**
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  dwt_rxreset();
  handle_rx_to();
  // printf("RX timeout\r\n");
}

/**
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  dwt_rxreset();
  printf("RX error\r\n");
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/**
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  handle_tx();
}

/* Local functions */
/**
 * @brief Interrupt handler calls the DW1000 ISR API. Call back corresponding to each event defined in ss_init_main
 * 
 * @param pin 
 * @param action 
 */
void vInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  dwt_isr(); // DW1000 interrupt service routine 
}
