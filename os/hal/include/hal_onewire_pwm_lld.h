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
 * @brief   Driver configuration structure.
 */
#define onewire_lld_driver_fields                                               \
  /**                                                                           \
   * @brief Pointer to @p PWM driver used for communication.                    \
   */                                                                           \
  PWMDriver                 *pwmd;                                              \
   /**                                                                          \
   * @brief Pointer to configuration structure for underlying PWM driver.       \
   * @note  It is NOT constant because 1-wire driver needs to change them       \
   *        during normal functioning.                                          \
   */                                                                           \
  PWMConfig                 *pwmcfg;                                            \
  /**                                                                           \
   * @brief   Active logic level for master channel.                            \
   * @details Just set it to @p PWM_OUTPUT_ACTIVE_LOW when 1-wire bus           \
   *          connected to direct (not complementary) output of the timer.      \
   *          In opposite case you need to check documentation to choose        \
   *          correct value.                                                    \
   */                                                                           \
  pwmmode_t                 pwmmode;                                            \
  /**                                                                           \
   * @brief Number of PWM channel used as master pulse generator.               \
   */                                                                           \
  size_t                    master_channel;                                     \
  /**                                                                           \
   * @brief Number of PWM channel used as sample interrupt generator.           \
   */                                                                           \
  size_t                    sample_channel

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
//   void onewireObjectInit(onewireDriver *owp);
  void onewire_lld_start(onewireDriver *owp, const onewireConfig *config);
  void onewire_lld_stop(onewireDriver *owp);
  bool onewire_lld_reset(onewireDriver *owp);
  void onewire_lld_read(onewireDriver *owp, uint8_t *rxbuf, size_t rxbytes);
//   uint8_t onewireCRC(const uint8_t *buf, size_t len);
  void onewire_lld_write(onewireDriver *owp, uint8_t *txbuf,
                    size_t txbytes, systime_t pullup_time);
// #if ONEWIRE_USE_SEARCH_ROM
//   size_t onewireSearchRom(onewireDriver *owp,
//                           uint8_t *result, size_t max_rom_cnt);
// #endif /* ONEWIRE_USE_SEARCH_ROM */
// #if ONEWIRE_SYNTH_SEARCH_TEST
//   void _synth_ow_write_bit(onewireDriver *owp, ioline_t bit);
//   ioline_t _synth_ow_read_bit(void);
//   void synthSearchRomTest(onewireDriver *owp);
// #endif /* ONEWIRE_SYNTH_SEARCH_TEST */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ONEWIRE && ONEWIRE_USE_PWM */

#endif /* HAL_ONEWIRE_PWM_LLD_H_ */

/** @} */









