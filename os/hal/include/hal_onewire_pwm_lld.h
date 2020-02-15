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

#ifndef HAL_ONEWIRE_PWM_LLD_H_
#define HAL_ONEWIRE_PWM_LLD_H_

#if (HAL_USE_ONEWIRE && ONEWIRE_USE_PWM) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/
#if !HAL_USE_PWM
#error "1-wire PWM Driver requires HAL_USE_PWM"
#endif

#if !HAL_USE_PAL
#error "1-wire PWM Driver requires HAL_USE_PAL"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   LLD-specific configuration fields.
 */
#define onewire_lld_config_fields                                             \
  /**                                                                         \
   * @brief Pointer to @p PWM driver used for communication.                  \
   */                                                                         \
  PWMDriver                 *pwmd;                                            \
   /**                                                                        \
   * @brief Pointer to configuration structure for underlying PWM driver.     \
   * @note  It is NOT constant because 1-wire driver needs to change them     \
   *        during normal functioning.                                        \
   */                                                                         \
  PWMConfig                 *pwmcfg;                                          \
  /**                                                                         \
   * @brief   Active logic level for master channel.                          \
   * @details Just set it to @p PWM_OUTPUT_ACTIVE_LOW when 1-wire bus         \
   *          connected to direct (not complementary) output of the timer.    \
   *          In opposite case you need to check documentation to choose      \
   *          correct value.                                                  \
   */                                                                         \
  pwmmode_t                 pwmmode;                                          \
  /**                                                                         \
   * @brief Number of PWM channel used as master pulse generator.             \
   */                                                                         \
  size_t                    master_channel;                                   \
  /**                                                                         \
   * @brief Number of PWM channel used as sample interrupt generator.         \
   */                                                                         \
  size_t                    sample_channel

/**
 * @brief   LLD-specific driver fields.
 */
#define onewire_lld_driver_fields                                             \
  /**                                                                         \
   * @brief Bit read callback, stored internally for use in IRQ handler.      \
   */                                                                         \
  onewire_read_callback_t   read_cb;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

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









