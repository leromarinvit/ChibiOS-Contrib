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

/*===========================================================================*/
/* Main ideas:                                                               */
/*===========================================================================

1) switch PWM output pin to open drain mode.
2) start 2 channels _simultaneously_. First (master channel) generates
   pulses (read time slots) second (sample channel) generates interrupts
   from where read pin function will be called.

-      --------------------------------------- master channel generates pulses
 |   /                            .
  --.............................  <---------- slave (not)pulls down bus here
-             -------------------------------- sample channel reads pad state
 |            |
  -------------
              ^
              | read interrupt fires here

For data write it is only master channel needed. Data bit width updates
on every timer overflow event.
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

#if (HAL_USE_ONEWIRE && ONEWIRE_USE_PWM) || defined(__DOXYGEN__)

#include <string.h>
#include <limits.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
/**
 * @brief     1MHz clock for PWM driver.
 */
#define ONEWIRE_PWM_FREQUENCY         1000000

/**
 * @brief     Local function declarations.
 */
static void ow_reset_cb(PWMDriver *pwmp, onewireDriver *owp);
static void pwm_reset_cb(PWMDriver *pwmp);
static void pwm_read_bit_cb(PWMDriver *pwmp);
static void ow_write_bit_cb(PWMDriver *pwmp, onewireDriver *owp);
static void pwm_write_bit_cb(PWMDriver *pwmp);

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
#if defined(STM32F1XX)
  palSetPadMode(owp->config->port, owp->config->pad,
      owp->config->pad_mode_idle);
#endif
  pwmStop(owp->config->pwmd);
}

/**
 * @brief     Put bus in active mode.
 */
static void ow_bus_active(onewireDriver *owp) {
  pwmStart(owp->config->pwmd, owp->config->pwmcfg);
#if defined(STM32F1XX)
  palSetPadMode(owp->config->port, owp->config->pad,
      owp->config->pad_mode_active);
#endif
}

/**
 * @brief     Function performing read of single bit.
 * @note      It must be callable from any context.
 */
static bool ow_read_bit(onewireDriver *owp) {
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
static void ow_write_bit_I(onewireDriver *owp, bool bit) {
#if ONEWIRE_SYNTH_SEARCH_TEST
  _synth_ow_write_bit(owp, bit);
#else
  osalSysLockFromISR();
  if (0 == bit) {
    pwmEnableChannelI(owp->config->pwmd, owp->config->master_channel,
                      ONEWIRE_ZERO_WIDTH);
  }
  else {
    pwmEnableChannelI(owp->config->pwmd, owp->config->master_channel,
                      ONEWIRE_ONE_WIDTH);
  }
  osalSysUnlockFromISR();
#endif
}

/**
 * @brief     PWM adapter
 */
static void pwm_reset_cb(PWMDriver *pwmp) {
  ow_reset_cb(pwmp, &OWD1);
}

/**
 * @brief     PWM adapter
 */
static void pwm_read_bit_cb(PWMDriver *pwmp) {
  switch (OWD1.read_cb(&OWD1)) {
  case ONEWIRE_READ_CB_ONE:
    ow_write_bit_I(&OWD1, 1);
    break;
  case ONEWIRE_READ_CB_ZERO:
    ow_write_bit_I(&OWD1, 0);
    break;
  case ONEWIRE_READ_CB_END:
    osalSysLockFromISR();
    pwmDisableChannelI(pwmp, OWD1.config->master_channel);
    pwmDisableChannelI(pwmp, OWD1.config->sample_channel);
    osalThreadResumeI(&OWD1.thread, MSG_OK);
    osalSysUnlockFromISR();
    break;
  }
}

/**
 * @brief     PWM adapter
 */
static void pwm_write_bit_cb(PWMDriver *pwmp) {
  ow_write_bit_cb(pwmp, &OWD1);
}

/**
 * @brief     1-wire reset pulse callback.
 * @note      Must be called from PWM's ISR.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @notapi
 */
static void ow_reset_cb(PWMDriver *pwmp, onewireDriver *owp) {

  owp->reg.slave_present = (PAL_LOW == ow_read_bit(owp));
  osalSysLockFromISR();
  pwmDisableChannelI(pwmp, owp->config->sample_channel);
  osalThreadResumeI(&owp->thread, MSG_OK);
  osalSysUnlockFromISR();
}

/**
 * @brief     1-wire bit transmission callback.
 * @note      Must be called from PWM's ISR.
 *
 * @param[in] pwmp      pointer to the @p PWMDriver object
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @notapi
 */
static void ow_write_bit_cb(PWMDriver *pwmp, onewireDriver *owp) {

  if (8 == owp->reg.bit) {
    owp->buf++;
    owp->reg.bit = 0;
    owp->reg.bytes--;

    if (0 == owp->reg.bytes) {
      osalSysLockFromISR();
      pwmDisableChannelI(pwmp, owp->config->master_channel);
      osalSysUnlockFromISR();
      /* used to prevent premature timer stop from userspace */
      owp->reg.final_timeslot = true;
      return;
    }
  }

  /* wait until timer generate last pulse */
  if (true == owp->reg.final_timeslot) {
    #if ONEWIRE_USE_STRONG_PULLUP
    if (owp->reg.need_pullup) {
      owp->reg.state = ONEWIRE_PULL_UP;
      owp->config->pullup_assert();
      owp->reg.need_pullup = false;
    }
    #endif

    osalSysLockFromISR();
    osalThreadResumeI(&owp->thread, MSG_OK);
    osalSysUnlockFromISR();
    return;
  }

  ow_write_bit_I(owp, (*owp->buf >> owp->reg.bit) & 1);
  owp->reg.bit++;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Activates the 1-wire driver.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @api
 */
void onewire_lld_start(onewireDriver *owp) {

  owp->config->pwmcfg->frequency = ONEWIRE_PWM_FREQUENCY;
  owp->config->pwmcfg->period = ONEWIRE_RESET_TOTAL_WIDTH;

#if !defined(STM32F1XX)
  palSetPadMode(owp->config->port, owp->config->pad,
      owp->config->pad_mode_active);
#endif
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
  pwmStop(owp->config->pwmd);
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
  PWMDriver *pwmd;
  PWMConfig *pwmcfg;
  size_t mch, sch;

  /* short circuit on bus or any other device transmit data */
  if (PAL_LOW == ow_read_bit(owp))
    return false;

  pwmd = owp->config->pwmd;
  pwmcfg = owp->config->pwmcfg;
  mch = owp->config->master_channel;
  sch = owp->config->sample_channel;


  pwmcfg->period = ONEWIRE_RESET_LOW_WIDTH + ONEWIRE_RESET_SAMPLE_WIDTH;
  pwmcfg->callback = NULL;
  pwmcfg->channels[mch].callback = NULL;
  pwmcfg->channels[mch].mode = owp->config->pwmmode;
  pwmcfg->channels[sch].callback = pwm_reset_cb;
  pwmcfg->channels[sch].mode = PWM_OUTPUT_DISABLED;

  ow_bus_active(owp);

  osalSysLock();
  pwmEnableChannelI(pwmd, mch, ONEWIRE_RESET_LOW_WIDTH);
  pwmEnableChannelI(pwmd, sch, ONEWIRE_RESET_SAMPLE_WIDTH);
  pwmEnableChannelNotificationI(pwmd, sch);
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  ow_bus_idle(owp);

  /* wait until slave release bus to discriminate short circuit condition */
  osalThreadSleepMicroseconds(500);
  return (PAL_HIGH == ow_read_bit(owp)) && (true == owp->reg.slave_present);
}

/**
 * @brief     Read data from slave device.
 *            Generates read pulses and calls the callback function at the
 *            sampling instant. This callback must do the actual read, and its
 *            return value determines if and how the process continues.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[in] cb        bit read callback
 */
void onewire_lld_read(onewireDriver *owp, onewire_read_callback_t cb) {
  PWMDriver *pwmd;
  PWMConfig *pwmcfg;
  size_t mch, sch;

  pwmd = owp->config->pwmd;
  pwmcfg = owp->config->pwmcfg;
  mch = owp->config->master_channel;
  sch = owp->config->sample_channel;

  pwmcfg->period = ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH;
  pwmcfg->callback = NULL;
  pwmcfg->channels[mch].callback = NULL;
  pwmcfg->channels[mch].mode = owp->config->pwmmode;
  pwmcfg->channels[sch].callback = pwm_read_bit_cb;
  pwmcfg->channels[sch].mode = PWM_OUTPUT_DISABLED;

  owp->read_cb = cb;

  ow_bus_active(owp);
  osalSysLock();
  pwmEnableChannelI(pwmd, mch, ONEWIRE_ONE_WIDTH);
  pwmEnableChannelI(pwmd, sch, ONEWIRE_SAMPLE_WIDTH);
  pwmEnableChannelNotificationI(pwmd, sch);
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  ow_bus_idle(owp);
}

/**
 * @brief     Write some bytes to slave device.
 *
 * @param[in] owp           pointer to the @p onewireDriver object
 */
void onewire_lld_write(onewireDriver *owp) {
  PWMDriver *pwmd;
  PWMConfig *pwmcfg;
  size_t mch, sch;

  pwmd = owp->config->pwmd;
  pwmcfg = owp->config->pwmcfg;
  mch = owp->config->master_channel;
  sch = owp->config->sample_channel;

  pwmcfg->period = ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH;
  pwmcfg->callback = pwm_write_bit_cb;
  pwmcfg->channels[mch].callback = NULL;
  pwmcfg->channels[mch].mode = owp->config->pwmmode;
  pwmcfg->channels[sch].callback = NULL;
  pwmcfg->channels[sch].mode = PWM_OUTPUT_DISABLED;

  ow_bus_active(owp);
  osalSysLock();
  pwmEnablePeriodicNotificationI(pwmd);
  osalThreadSuspendS(&owp->thread);
  osalSysUnlock();

  pwmDisablePeriodicNotification(pwmd);
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
