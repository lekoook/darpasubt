#ifndef _LOW_TIMER_H
#define _LOW_TIMER_H

#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "app_error.h"

/* Public function prototypes */
void lowTimerInit(void);
void lowTimerRepeatCreate(const app_timer_id_t *timerId, app_timer_timeout_handler_t handler);
void lowTimerSingleCreate(const app_timer_id_t *timerId, app_timer_timeout_handler_t handler);
void lowTimerStart(app_timer_id_t timerId, uint32_t timeout, void* p_context);
void lowTimerStop(app_timer_id_t timerId);
void lowTimerStopAll(void);

#endif