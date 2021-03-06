/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess,
                               2020 Oliver Hanser

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
 * @file    hal_onewire_delay_lld.c
 * @brief   1-wire Delay Driver code.
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
 * @details   Write bit on GPIO line. Blocks for bit length + recovery time.
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
  const int delay = bit ? ONEWIRE_ONE_WIDTH : ONEWIRE_ZERO_WIDTH;
  palClearPad(owp->config->port, owp->config->pad);
  owp->config->udelay(delay);
  palSetPad(owp->config->port, owp->config->pad);
  owp->config->udelay(ONEWIRE_ZERO_WIDTH + ONEWIRE_RECOVERY_WIDTH - delay);
#endif
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
  ow_bus_idle(owp);
}

/**
 * @brief   Deactivates the 1-wire driver.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @api
 */
void onewire_lld_stop(onewireDriver *owp) {
  ow_bus_idle(owp);
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
  if (PAL_LOW == ow_read_bit(owp))
    return false;

  ow_bus_active(owp);

  palClearPad(owp->config->port, owp->config->pad);
  owp->config->udelay(ONEWIRE_RESET_LOW_WIDTH);
  palSetPad(owp->config->port, owp->config->pad);
  owp->config->udelay(ONEWIRE_RESET_SAMPLE_WIDTH - ONEWIRE_RESET_LOW_WIDTH);
  owp->reg.slave_present = (PAL_LOW == ow_read_bit(owp));

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
  onewire_read_cb_result_t result = ONEWIRE_READ_CB_ONE;

  ow_bus_active(owp);

  do {
    onewire_read_cb_result_t last_result = result;
    palClearPad(owp->config->port, owp->config->pad);
    owp->config->udelay(ONEWIRE_ONE_WIDTH);
    if (last_result != ONEWIRE_READ_CB_ZERO)
      palSetPad(owp->config->port, owp->config->pad);
    owp->config->udelay(ONEWIRE_SAMPLE_WIDTH - ONEWIRE_ONE_WIDTH);
    result = cb(owp);
    if (last_result == ONEWIRE_READ_CB_ZERO) {
      owp->config->udelay(ONEWIRE_ZERO_WIDTH - ONEWIRE_SAMPLE_WIDTH);
      palSetPad(owp->config->port, owp->config->pad);
      owp->config->udelay(ONEWIRE_RECOVERY_WIDTH);
    } else {
      owp->config->udelay(ONEWIRE_RECOVERY_WIDTH + ONEWIRE_ZERO_WIDTH - ONEWIRE_SAMPLE_WIDTH);
    }
  } while (result != ONEWIRE_READ_CB_END);

  ow_bus_idle(owp);
}

/**
 * @brief     Write some bytes to slave device.
 *
 * @param[in] owp           pointer to the @p onewireDriver object
 */
void onewire_lld_write(onewireDriver *owp) {
  ow_bus_active(owp);

  while (owp->reg.bytes) {
    for (size_t i = 0; i < 8; i++)
      ow_write_bit_I(owp, (*owp->txbuf >> i) & 1);
    owp->txbuf++;
    owp->reg.bytes--;
  }

  ow_bus_idle(owp);
}

/*
 * Include test code (if enabled).
 */
#if ONEWIRE_SYNTH_SEARCH_TEST
#include "synth_searchrom.c"
#endif

#endif /* HAL_USE_ONEWIRE && ONEWIRE_USE_DELAY */

/** @} */
