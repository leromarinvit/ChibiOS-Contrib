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

/**
 * @file    hal_onewire_pwm_lld.h
 * @brief   1-wire PWM LLD macros and structures.
 *
 * @addtogroup onewire
 * @{
 */

#ifndef HAL_ONEWIRE_DELAY_LLD_H_
#define HAL_ONEWIRE_DELAY_LLD_H_

#if (HAL_USE_ONEWIRE && ONEWIRE_USE_DELAY) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

typedef void (*onewire_udelay_t)(uint32_t delay);

/**
 * @brief   Driver configuration structure.
 */
#define onewire_lld_config_fields                                             \
  /**                                                                         \
   * @brief Microsecond delay.                                                \
   */                                                                         \
  onewire_udelay_t          udelay

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

// extern onewireDriver OWD1;

#ifdef __cplusplus
extern "C" {
#endif
  void onewire_lld_start(onewireDriver *owp);
  void onewire_lld_stop(onewireDriver *owp);
  bool onewire_lld_reset(onewireDriver *owp);
  void onewire_lld_read(onewireDriver *owp, onewire_read_callback_t cb);
  void onewire_lld_write(onewireDriver *owp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ONEWIRE && ONEWIRE_USE_PWM */

#endif /* HAL_ONEWIRE_PWM_LLD_H_ */

/** @} */









