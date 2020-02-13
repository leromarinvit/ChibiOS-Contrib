/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

// /*===========================================================================*/
// /* General recommendations for strong pull usage                             */
// /*===========================================================================
//  * 1) Use separate power rail instead of strong pull up whenever possible.
//  *    Driver's strong pull up feature is very sensible to interrupt jitter.
//  * 2) Use specialized 1-wire bus master (DS2484 for example) if you are
//  *    forced to handle bus requiring strong pull up feature.
//  */

/**
 * @file    hal_onewire_pwm_lld.c
 * @brief   1-wire PWM Driver code.
 *
 * @addtogroup onewire
 * @{
 */

#include "hal.h"

#if (HAL_USE_ONEWIRE && ONEWIRE_USE_DELAY) || defined(__DOXYGEN__)

#include <string.h>
#include <limits.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
/**
 * @brief     Pulse width constants in microseconds.
 * @details   Inspired by Microchip's AN1199
 *            "1-Wire® Communication with PIC® Microcontroller"
 */
#define ONEWIRE_ZERO_WIDTH            60
#define ONEWIRE_ONE_WIDTH             6
#define ONEWIRE_SAMPLE_WIDTH          15
#define ONEWIRE_RECOVERY_WIDTH        10
#define ONEWIRE_RESET_LOW_WIDTH       480
#define ONEWIRE_RESET_SAMPLE_WIDTH    550
#define ONEWIRE_RESET_TOTAL_WIDTH     960

#ifndef ONEWIRE_UDELAY
#define ONEWIRE_UDELAY(t)             osalSysPolledDelayX(OSAL_US2RTC(STM32_SYSCLK, t))
#endif

/**
 * @brief     Local function declarations.
 */
// static void ow_reset_cb(PWMDriver *pwmp, onewireDriver *owp);
// static void pwm_reset_cb(PWMDriver *pwmp);
// static void pwm_read_bit_cb(PWMDriver *pwmp);
// static void ow_write_bit_cb(PWMDriver *pwmp, onewireDriver *owp);
// static void pwm_write_bit_cb(PWMDriver *pwmp);

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/**
 * @brief     Put bus in idle mode.
 */
static void ow_bus_idle(onewireDriver *owp) {
  palSetPad(owp->config->port, owp->config->pad);
  palSetPadMode(owp->config->port, owp->config->pad,
      owp->config->pad_mode_idle);
}

/**
 * @brief     Put bus in active mode.
 */
static void ow_bus_active(onewireDriver *owp) {
  palSetPad(owp->config->port, owp->config->pad);
  palSetPadMode(owp->config->port, owp->config->pad,
      owp->config->pad_mode_active);
}

/**
 * @brief     Function performing read of single bit.
 * @note      It must be callable from any context.
 */
bool onewire_lld_read_bit(onewireDriver *owp) {
#if ONEWIRE_SYNTH_SEARCH_TEST
  (void)owp;
  return _synth_ow_read_bit();
#else
  return palReadPad(owp->config->port, owp->config->pad);
#endif
}

/**
 * @brief     Write bit routine.
 * @details   Switch PWM channel to 'width' or 'narrow' pulse depending
 *            on value of bit need to be transmitted.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[in] bit       value to be written
 *
 * @notapi
 */
void onewire_lld_write_bit_I(onewireDriver *owp, bool bit) {
#if ONEWIRE_SYNTH_SEARCH_TEST
  _synth_ow_write_bit(owp, bit);
#else
  const int delay = bit ? ONEWIRE_ONE_WIDTH : ONEWIRE_ZERO_WIDTH;
  palClearPad(owp->config->port, owp->config->pad);
  ONEWIRE_UDELAY(delay);
  palSetPad(owp->config->port, owp->config->pad);
  ONEWIRE_UDELAY(ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH - delay);
  // osalSysLockFromISR();
  // if (0 == bit) {
  //   pwmEnableChannelI(owp->config->pwmd, owp->config->master_channel,
  //                     ONEWIRE_ZERO_WIDTH);
  // }
  // else {
  //   pwmEnableChannelI(owp->config->pwmd, owp->config->master_channel,
  //                     ONEWIRE_ONE_WIDTH);
  // }
  // osalSysUnlockFromISR();
#endif
}

// /**
//  * @brief     1-wire reset pulse callback.
//  * @note      Must be called from PWM's ISR.
//  *
//  * @param[in] pwmp      pointer to the @p PWMDriver object
//  * @param[in] owp       pointer to the @p onewireDriver object
//  *
//  * @notapi
//  */
// static void ow_reset_cb(PWMDriver *pwmp, onewireDriver *owp) {

//   owp->reg.slave_present = (PAL_LOW == onewire_lld_read_bit(owp));
//   osalSysLockFromISR();
//   pwmDisableChannelI(pwmp, owp->config->sample_channel);
//   osalThreadResumeI(&owp->thread, MSG_OK);
//   osalSysUnlockFromISR();
// }

// /**
//  * @brief     1-wire bit transmission callback.
//  * @note      Must be called from PWM's ISR.
//  *
//  * @param[in] pwmp      pointer to the @p PWMDriver object
//  * @param[in] owp       pointer to the @p onewireDriver object
//  *
//  * @notapi
//  */
// static void ow_write_bit_cb(PWMDriver *pwmp, onewireDriver *owp) {

//   if (8 == owp->reg.bit) {
//     owp->buf++;
//     owp->reg.bit = 0;
//     owp->reg.bytes--;

//     if (0 == owp->reg.bytes) {
//       osalSysLockFromISR();
//       pwmDisableChannelI(pwmp, owp->config->master_channel);
//       osalSysUnlockFromISR();
//       /* used to prevent premature timer stop from userspace */
//       owp->reg.final_timeslot = true;
//       return;
//     }
//   }

//   /* wait until timer generate last pulse */
//   if (true == owp->reg.final_timeslot) {
//     #if ONEWIRE_USE_STRONG_PULLUP
//     if (owp->reg.need_pullup) {
//       owp->reg.state = ONEWIRE_PULL_UP;
//       owp->config->pullup_assert();
//       owp->reg.need_pullup = false;
//     }
//     #endif

//     osalSysLockFromISR();
//     osalThreadResumeI(&owp->thread, MSG_OK);
//     osalSysUnlockFromISR();
//     return;
//   }

//   onewire_lld_write_bit_I(owp, (*owp->buf >> owp->reg.bit) & 1);
//   owp->reg.bit++;
// }

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Configures and activates the 1-wire driver.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[in] config    pointer to the @p onewireConfig object
 *
 * @api
 */
void onewire_lld_start(onewireDriver *owp) {

// #if !defined(STM32F1XX)
//   palSetPadMode(owp->config->port, owp->config->pad,
//       owp->config->pad_mode_active);
// #endif
  ow_bus_idle(owp);
}

/**
 * @brief   Deactivates the UART peripheral.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @api
 */
void onewire_lld_stop(onewireDriver *owp) {
  ow_bus_idle(owp);
  // pwmStop(owp->config->pwmd);
}

/**
 * @brief     Generate reset pulse on bus.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @return              Bool flag denoting device presence.
 * @retval true         There is at least one device on bus.
 */
bool onewire_lld_reset(onewireDriver *owp) {

  /* short circuit on bus or any other device transmit data */
  if (PAL_LOW == onewire_lld_read_bit(owp))
    return false;

  ow_bus_active(owp);

  palClearPad(owp->config->port, owp->config->pad);
  ONEWIRE_UDELAY(ONEWIRE_RESET_LOW_WIDTH);
  palSetPad(owp->config->port, owp->config->pad);
  ONEWIRE_UDELAY(ONEWIRE_RESET_SAMPLE_WIDTH - ONEWIRE_RESET_LOW_WIDTH);
  owp->reg.slave_present = (PAL_LOW == onewire_lld_read_bit(owp));

  ow_bus_idle(owp);

  /* wait until slave release bus to discriminate short circuit condition */
  osalThreadSleepMicroseconds(500);
  return (PAL_HIGH == onewire_lld_read_bit(owp)) && (true == owp->reg.slave_present);
}

/**
 * @brief     Read some bytes from slave device.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[out] rxbuf    pointer to the buffer for read data
 * @param[in] rxbytes   amount of data to be received
 */
void onewire_lld_read(onewireDriver *owp, onewire_read_callback_t cb) {
  // PWMDriver *pwmd;
  // PWMConfig *pwmcfg;
  // size_t mch, sch;
  // return;
  onewire_read_cb_result_t result = ONEWIRE_READ_CB_ONE;

  // pwmd = owp->config->pwmd;
  // pwmcfg = owp->config->pwmcfg;
  // mch = owp->config->master_channel;
  // sch = owp->config->sample_channel;

  // pwmcfg->period = ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH;
  // pwmcfg->callback = NULL;
  // pwmcfg->channels[mch].callback = NULL;
  // pwmcfg->channels[mch].mode = owp->config->pwmmode;
  // pwmcfg->channels[sch].callback = pwm_read_bit_cb;
  // pwmcfg->channels[sch].mode = PWM_OUTPUT_DISABLED;

  // owp->read_cb = cb;

  ow_bus_active(owp);

  do {
    onewire_read_cb_result_t last_result = result;
    palClearPad(owp->config->port, owp->config->pad);
    ONEWIRE_UDELAY(ONEWIRE_ONE_WIDTH);
    if (last_result != ONEWIRE_READ_CB_ZERO)
      palSetPad(owp->config->port, owp->config->pad);
    ONEWIRE_UDELAY(ONEWIRE_SAMPLE_WIDTH - ONEWIRE_ONE_WIDTH);
    result = cb(owp);
    if (last_result == ONEWIRE_READ_CB_ZERO) {
      ONEWIRE_UDELAY(ONEWIRE_ZERO_WIDTH - ONEWIRE_SAMPLE_WIDTH);
      palSetPad(owp->config->port, owp->config->pad);
      ONEWIRE_UDELAY(ONEWIRE_RECOVERY_WIDTH);
    } else {
      ONEWIRE_UDELAY(ONEWIRE_RECOVERY_WIDTH + ONEWIRE_ZERO_WIDTH - ONEWIRE_SAMPLE_WIDTH);
    }
  } while (result != ONEWIRE_READ_CB_END);


  // osalSysLock();
  // pwmEnableChannelI(pwmd, mch, ONEWIRE_ONE_WIDTH);
  // pwmEnableChannelI(pwmd, sch, ONEWIRE_SAMPLE_WIDTH);
  // pwmEnableChannelNotificationI(pwmd, sch);
  // osalThreadSuspendS(&owp->thread);
  // osalSysUnlock();

  ow_bus_idle(owp);
}

/**
 * @brief     Write some bytes to slave device.
 *
 * @param[in] owp           pointer to the @p onewireDriver object
 * @param[in] txbuf         pointer to the buffer with data to be written
 * @param[in] txbytes       amount of data to be written
 * @param[in] pullup_time   how long strong pull up must be activated. Set
 *                          it to 0 if not needed.
 */
void onewire_lld_write(onewireDriver *owp) {
  // PWMDriver *pwmd;
  // PWMConfig *pwmcfg;
  // size_t mch, sch;

  // pwmd = owp->config->pwmd;
  // pwmcfg = owp->config->pwmcfg;
  // mch = owp->config->master_channel;
  // sch = owp->config->sample_channel;

  // pwmcfg->period = ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH;
  // pwmcfg->callback = pwm_write_bit_cb;
  // pwmcfg->channels[mch].callback = NULL;
  // pwmcfg->channels[mch].mode = owp->config->pwmmode;
  // pwmcfg->channels[sch].callback = NULL;
  // pwmcfg->channels[sch].mode = PWM_OUTPUT_DISABLED;

  ow_bus_active(owp);

  while (owp->reg.bytes) {
    for (size_t i = 0; i < 8; i++)
      onewire_lld_write_bit_I(owp, (*owp->buf >> i) & 1);
    owp->buf++;
    owp->reg.bytes--;
  }
  // osalSysLock();
  // pwmEnablePeriodicNotificationI(pwmd);
  // osalThreadSuspendS(&owp->thread);
  // osalSysUnlock();

  // pwmDisablePeriodicNotification(pwmd);
  ow_bus_idle(owp);
}

/*
 * Include test code (if enabled).
 */
#if ONEWIRE_SYNTH_SEARCH_TEST
#include "synth_searchrom.c"
#endif

#endif /* HAL_USE_ONEWIRE && ONEWIRE_USE_PWM */

/** @} */
