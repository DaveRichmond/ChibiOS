/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    can.c
 * @brief   CAN Driver code.
 *
 * @addtogroup CAN
 * @{
 */

#include "hal.h"

#if (HAL_USE_CAN == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   CAN Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void canInit(void) {

  can_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p CANDriver structure.
 *
 * @param[out] canp     pointer to the @p CANDriver object
 *
 * @init
 */
void canObjectInit(CANDriver *canp) {

  canp->state    = CAN_STOP;
  canp->config   = NULL;
  osalThreadQueueObjectInit(&canp->txqueue);
  osalThreadQueueObjectInit(&canp->rxqueue);
  osalEventObjectInit(&canp->rxfull_event);
  osalEventObjectInit(&canp->txempty_event);
  osalEventObjectInit(&canp->error_event);
#if CAN_USE_SLEEP_MODE == TRUE
  osalEventObjectInit(&canp->sleep_event);
  osalEventObjectInit(&canp->wakeup_event);
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 * @note    Activating the CAN bus can be a slow operation.
 * @note    Unlike other drivers it is not possible to restart the CAN
 *          driver without first stopping it using canStop().
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] config    pointer to the @p CANConfig object. Depending on
 *                      the implementation the value can be @p NULL.
 *
 * @api
 */
void canStart(CANDriver *canp, const CANConfig *config) {

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert(canp->state == CAN_STOP, "invalid state");

  /* Entering initialization mode. */
  canp->state = CAN_STARTING;
  canp->config = config;

  /* Low level initialization, could be a slow process and sleeps could
     be performed inside.*/
  can_lld_start(canp);

  /* The driver finally goes into the ready state.*/
  canp->state = CAN_READY;
  osalSysUnlock();
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @api
 */
void canStop(CANDriver *canp) {

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert((canp->state == CAN_STOP) || (canp->state == CAN_READY),
                "invalid state");

  /* The low level driver is stopped.*/
  can_lld_stop(canp);
  canp->state  = CAN_STOP;

  /* Threads waiting on CAN APIs are notified that the driver has been
     stopped in order to not have stuck threads.*/
  osalThreadDequeueAllI(&canp->rxqueue, MSG_RESET);
  osalThreadDequeueAllI(&canp->txqueue, MSG_RESET);
  osalOsRescheduleS();
  osalSysUnlock();
}

/**
 * @brief   Can frame transmission.
 * @details The specified frame is queued for transmission, if the hardware
 *          queue is full then the invoking thread is queued.
 * @note    Trying to transmit while in sleep mode simply enqueues the thread.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation result.
 * @retval MSG_OK       the frame has been queued for transmission.
 * @retval MSG_TIMEOUT  The operation has timed out.
 * @retval MSG_RESET    The driver has been stopped while waiting.
 *
 * @api
 */
msg_t canTransmit(CANDriver *canp,
                  canmbx_t mailbox,
                  const CANTxFrame *ctfp,
                  systime_t timeout) {

  osalDbgCheck((canp != NULL) && (ctfp != NULL) &&
               (mailbox <= (canmbx_t)CAN_TX_MAILBOXES));

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");
  /*lint -save -e9007 [13.5] Right side is supposed to be pure.*/
  while ((canp->state == CAN_SLEEP) || !can_lld_is_tx_empty(canp, mailbox)) {
  /*lint -restore*/
   msg_t msg = osalThreadEnqueueTimeoutS(&canp->txqueue, timeout);
    if (msg != MSG_OK) {
      osalSysUnlock();
      return msg;
    }
  }
  can_lld_transmit(canp, mailbox, ctfp);
  osalSysUnlock();
  return MSG_OK;
}

/**
 * @brief   Can frame receive.
 * @details The function waits until a frame is received.
 * @note    Trying to receive while in sleep mode simply enqueues the thread.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout (useful in an
 *                        event driven scenario where a thread never blocks
 *                        for I/O).
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation result.
 * @retval MSG_OK       a frame has been received and placed in the buffer.
 * @retval MSG_TIMEOUT  The operation has timed out.
 * @retval MSG_RESET    The driver has been stopped while waiting.
 *
 * @api
 */
msg_t canReceive(CANDriver *canp,
                 canmbx_t mailbox,
                 CANRxFrame *crfp,
                 systime_t timeout) {

  osalDbgCheck((canp != NULL) && (crfp != NULL) &&
               (mailbox < (canmbx_t)CAN_RX_MAILBOXES));

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");
  /*lint -save -e9007 [13.5] Right side is supposed to be pure.*/
  while ((canp->state == CAN_SLEEP) || !can_lld_is_rx_nonempty(canp, mailbox)) {
  /*lint -restore*/
    msg_t msg = osalThreadEnqueueTimeoutS(&canp->rxqueue, timeout);
    if (msg != MSG_OK) {
      osalSysUnlock();
      return msg;
    }
  }
  can_lld_receive(canp, mailbox, crfp);
  osalSysUnlock();
  return MSG_OK;
}

#if (CAN_USE_SLEEP_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 * @details This function puts the CAN driver in sleep mode and broadcasts
 *          the @p sleep_event event source.
 * @pre     In order to use this function the option @p CAN_USE_SLEEP_MODE must
 *          be enabled and the @p CAN_SUPPORTS_SLEEP mode must be supported
 *          by the low level driver.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @api
 */
void canSleep(CANDriver *canp) {

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");
  if (canp->state == CAN_READY) {
    can_lld_sleep(canp);
    canp->state = CAN_SLEEP;
    osalEventBroadcastFlagsI(&canp->sleep_event, (eventflags_t)0);
    osalOsRescheduleS();
  }
  osalSysUnlock();
}

/**
 * @brief   Enforces leaving the sleep mode.
 * @note    The sleep mode is supposed to be usually exited automatically by
 *          an hardware event.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 */
void canWakeup(CANDriver *canp) {

  osalDbgCheck(canp != NULL);

  osalSysLock();
  osalDbgAssert((canp->state == CAN_READY) || (canp->state == CAN_SLEEP),
                "invalid state");
  if (canp->state == CAN_SLEEP) {
    can_lld_wakeup(canp);
    canp->state = CAN_READY;
    osalEventBroadcastFlagsI(&canp->wakeup_event, (eventflags_t)0);
    osalOsRescheduleS();
  }
  osalSysUnlock();
}
#endif /* CAN_USE_SLEEP_MODE == TRUE */

#endif /* HAL_USE_CAN == TRUE */

/** @} */
