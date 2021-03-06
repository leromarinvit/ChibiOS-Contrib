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

   This aims to be a generic "upper half" 1-wire driver. The actual bit
   twiddling and timing generation is left to the lower half.

 * Write: Straightforward, just handle strong pullup here and defer the rest of
          the write to the LLD.
 * Read:  The LLD is expected to generate read pulses and call back in the right
          moment for the actual read. This complexity is needed to support the
          Search ROM algorithm without reimplementing it in every LLD. Note,
          however, that this approach is incompatible with hypothetical byte
          oriented (instead of bitwise) LLDs.
*/

/*===========================================================================*/
/* General recommendations for strong pull usage                             */
/*===========================================================================
 * 1) Use separate power rail instead of strong pull up whenever possible.
 *    Driver's strong pull up feature is very sensible to interrupt jitter.
 * 2) Use specialized 1-wire bus master (DS2484 for example) if you are
 *    forced to handle bus requiring strong pull up feature.
 */

/**
 * @file    hal_onewire.c
 * @brief   1-wire Driver code.
 *
 * @addtogroup onewire
 * @{
 */

#include "hal.h"

#if (HAL_USE_ONEWIRE == TRUE) || defined(__DOXYGEN__)

#include <string.h>
#include <limits.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief     Pulse width constants in microseconds.
 * @details   Inspired by Microchip's AN1199
 *            "1-Wire?? Communication with PIC?? Microcontroller"
 */
#define ONEWIRE_ZERO_WIDTH            60
#define ONEWIRE_ONE_WIDTH             6
#define ONEWIRE_SAMPLE_WIDTH          15
#define ONEWIRE_RECOVERY_WIDTH        10
#define ONEWIRE_RESET_LOW_WIDTH       480
#define ONEWIRE_RESET_SAMPLE_WIDTH    550
#define ONEWIRE_RESET_TOTAL_WIDTH     960

/**
 * @brief     Local function declarations.
 */
#if ONEWIRE_USE_SEARCH_ROM
static onewire_read_cb_result_t ow_search_rom_cb(onewireDriver *owp);
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/
/**
 * @brief 1-wire driver identifier.
 */
onewireDriver OWD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/
/**
 * @brief     Look up table for fast 1-wire CRC calculation
 */
static const uint8_t onewire_crc_table[256] = {
    0x0,  0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x1,  0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x3,  0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x2,  0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x7,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x6,  0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x4,  0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x5,  0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0xf,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0xe,  0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0xc,  0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0xd,  0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x8,  0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x9,  0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0xb,  0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0xa,  0x54, 0xd7, 0x89, 0x6b, 0x35
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief     Function performing read of single bit.
 * @note      It must be callable from any context.
 */
static ioline_t ow_read_bit(onewireDriver *owp) {
#if ONEWIRE_SYNTH_SEARCH_TEST
  (void)owp;
  return _synth_ow_read_bit();
#else
  return palReadPad(owp->config->port, owp->config->pad);
#endif
}

/**
 * @brief     1-wire read bit callback.
 * @note      Must be callable from any context.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @notapi
 */
static onewire_read_cb_result_t ow_read_bit_cb(onewireDriver *owp) {

  *owp->rxbuf |= ow_read_bit(owp) << owp->reg.bit;
  owp->reg.bit++;
  if (8 == owp->reg.bit) {
    owp->reg.bit = 0;
    owp->rxbuf++;
    owp->reg.bytes--;
    if (0 == owp->reg.bytes)
      return ONEWIRE_READ_CB_END;
  }
  return ONEWIRE_READ_CB_ONE;
}

#if ONEWIRE_USE_SEARCH_ROM
/**
 * @brief   Helper function for collision handler
 *
 * @param[in] sr        pointer to the @p onewire_search_rom_t helper structure
 * @param[in] bit       discovered bit to be stored in helper structure
 */
static void store_bit(onewire_search_rom_t *sr, uint8_t bit) {

  size_t rb = sr->reg.rombit;

  sr->retbuf[rb / CHAR_BIT] |= bit << (rb % CHAR_BIT);
  sr->reg.rombit++;
}

/**
 * @brief     Helper function for collision handler
 * @details   Extract bit from previous search path.
 *
 * @param[in] path      pointer to the array with previous path stored in
 *                      'search ROM' helper structure
 * @param[in] bit       number of bit [0..63]
 */
static uint8_t extract_path_bit(const uint8_t *path, size_t bit) {

  return (path[bit / CHAR_BIT] >> (bit % CHAR_BIT)) & 1;
}

/**
 * @brief     Collision handler for 'search ROM' procedure.
 * @details   You can find algorithm details in APPNOTE 187
 *            "1-Wire Search Algorithm" from Maxim
 *
 * @param[in,out] sr    pointer to the @p onewire_search_rom_t helper structure
 */
static uint8_t collision_handler(onewire_search_rom_t *sr) {

  uint8_t bit;

  switch(sr->reg.search_iter) {
  case ONEWIRE_SEARCH_ROM_NEXT:
    if ((int)sr->reg.rombit < sr->last_zero_branch) {
      bit = extract_path_bit(sr->prev_path, sr->reg.rombit);
      if (0 == bit) {
        sr->prev_zero_branch = sr->reg.rombit;
        sr->reg.result = ONEWIRE_SEARCH_ROM_SUCCESS;
      }
      store_bit(sr, bit);
      return bit;
    }
    else if ((int)sr->reg.rombit == sr->last_zero_branch) {
      sr->last_zero_branch = sr->prev_zero_branch;
      store_bit(sr, 1);
      return 1;
    }
    else {
      /* found next branch some levels deeper */
      sr->prev_zero_branch = sr->last_zero_branch;
      sr->last_zero_branch = sr->reg.rombit;
      store_bit(sr, 0);
      sr->reg.result = ONEWIRE_SEARCH_ROM_SUCCESS;
      return 0;
    }
    break;

  case ONEWIRE_SEARCH_ROM_FIRST:
    /* always take 0-branch */
    sr->prev_zero_branch = sr->last_zero_branch;
    sr->last_zero_branch = sr->reg.rombit;
    store_bit(sr, 0);
    sr->reg.result = ONEWIRE_SEARCH_ROM_SUCCESS;
    return 0;
    break;

  default:
    osalSysHalt("Unhandled case");
    return 0; /* warning supressor */
    break;
  }
}

/**
 * @brief     1-wire search ROM callback.
 * @note      Must be callable from any context.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @notapi
 */
static onewire_read_cb_result_t ow_search_rom_cb(onewireDriver *owp) {

  onewire_read_cb_result_t result = ONEWIRE_READ_CB_ONE;
  onewire_search_rom_t *sr = &owp->search_rom;

  if (0 == sr->reg.bit_step) {                    /* read direct bit */
    sr->reg.bit_buf |= ow_read_bit(owp);
    sr->reg.bit_step++;
  }
  else if (1 == sr->reg.bit_step) {               /* read complement bit */
    sr->reg.bit_buf |= ow_read_bit(owp) << 1;
    sr->reg.bit_step++;
    switch(sr->reg.bit_buf){
    case 0b11:
      /* no one device on bus or any other fail happened */
      sr->reg.result = ONEWIRE_SEARCH_ROM_ERROR;
      goto THE_END;
      break;
    case 0b01:
      /* all slaves have 1 in this position */
      store_bit(sr, 1);
      // onewire_lld_write_bit_I(owp, 1);
      result = ONEWIRE_READ_CB_ONE;
      break;
    case 0b10:
      /* all slaves have 0 in this position */
      store_bit(sr, 0);
      // onewire_lld_write_bit_I(owp, 0);
      result = ONEWIRE_READ_CB_ZERO;
      break;
    case 0b00:
      /* collision */
      sr->reg.single_device = false;
      // onewire_lld_write_bit_I(owp, collision_handler(sr));
      result = collision_handler(sr) ? ONEWIRE_READ_CB_ONE : ONEWIRE_READ_CB_ZERO;
      break;
    }
  }
  else {                                      /* start next step */
    #if !ONEWIRE_SYNTH_SEARCH_TEST
    // onewire_lld_write_bit_I(owp, 1);
    result = ONEWIRE_READ_CB_ONE;
    #endif
    sr->reg.bit_step = 0;
    sr->reg.bit_buf = 0;
  }

  /* one ROM successfully discovered */
  if (64 == sr->reg.rombit) {
    sr->reg.devices_found++;
    sr->reg.search_iter = ONEWIRE_SEARCH_ROM_NEXT;
    if (true == sr->reg.single_device)
      sr->reg.result = ONEWIRE_SEARCH_ROM_LAST;
    goto THE_END;
  }
  return result; /* next search bit iteration */

THE_END:
  return ONEWIRE_READ_CB_END;
}

/**
 * @brief       Helper function. Initialize structures required by 'search ROM'.
 * @details     Early reset. Call it once before 'search ROM' routine.
 *
 * @param[in] sr        pointer to the @p onewire_search_rom_t helper structure
 */
static void search_clean_start(onewire_search_rom_t *sr) {

  sr->reg.single_device = true; /* presume simplest way at beginning */
  sr->reg.result = ONEWIRE_SEARCH_ROM_LAST;
  sr->reg.search_iter = ONEWIRE_SEARCH_ROM_FIRST;
  sr->retbuf = NULL;
  sr->reg.devices_found = 0;
  memset(sr->prev_path, 0, 8);

  sr->reg.rombit = 0;
  sr->reg.bit_step = 0;
  sr->reg.bit_buf = 0;
  sr->last_zero_branch = -1;
  sr->prev_zero_branch = -1;
}

/**
 * @brief       Helper function. Prepare structures required by 'search ROM'.
 *
 * @param[in] sr        pointer to the @p onewire_search_rom_t helper structure
 */
static void search_clean_iteration(onewire_search_rom_t *sr) {

  sr->reg.rombit = 0;
  sr->reg.bit_step = 0;
  sr->reg.bit_buf = 0;
  sr->reg.result = ONEWIRE_SEARCH_ROM_LAST;
}
#endif /* ONEWIRE_USE_SEARCH_ROM */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Calculates 1-wire CRC.
 *
 * @param[in] buf     pointer to the data buffer
 * @param[in] len     lenght of data buffer
 *
 * @init
 */
uint8_t onewireCRC(const uint8_t *buf, size_t len) {
  uint8_t ret = 0;
  size_t i;

  for (i=0; i<len; i++)
    ret = onewire_crc_table[ret ^ buf[i]];

  return ret;
}

/**
 * @brief   Initializes @p onewireDriver structure.
 *
 * @param[out] owp    pointer to the @p onewireDriver object
 *
 * @init
 */
void onewireObjectInit(onewireDriver *owp) {

  osalDbgCheck(NULL != owp);

  owp->config = NULL;
  owp->reg.slave_present = false;
  owp->reg.state = ONEWIRE_STOP;
  owp->thread = NULL;

  owp->reg.bytes = 0;
  owp->reg.bit = 0;
  owp->reg.final_timeslot = false;
  owp->txbuf = NULL;

#if ONEWIRE_USE_STRONG_PULLUP
  owp->reg.need_pullup = false;
#endif
}

/**
 * @brief   Configures and activates the 1-wire driver.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[in] config    pointer to the @p onewireConfig object
 *
 * @api
 */
void onewireStart(onewireDriver *owp, const onewireConfig *config) {

  osalDbgCheck((NULL != owp) && (NULL != config));
  osalDbgAssert(ONEWIRE_STOP == owp->reg.state, "Invalid state");
#if ONEWIRE_USE_STRONG_PULLUP
  osalDbgCheck((NULL != config->pullup_assert) &&
               (NULL != config->pullup_release));
#endif

  owp->config = config;

  onewire_lld_start(owp);

  owp->reg.state = ONEWIRE_READY;
}

/**
 * @brief   Deactivates the onewire peripheral.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @api
 */
void onewireStop(onewireDriver *owp) {
  osalDbgCheck(NULL != owp);
#if ONEWIRE_USE_STRONG_PULLUP
  owp->config->pullup_release();
#endif
  onewire_lld_stop(owp);
  owp->config = NULL;
  owp->reg.state = ONEWIRE_STOP;
}

/**
 * @brief     Generate reset pulse on bus.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 *
 * @return              Bool flag denoting device presence.
 * @retval true         There is at least one device on bus.
 */
bool onewireReset(onewireDriver *owp) {

  osalDbgCheck(NULL != owp);
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");

  return onewire_lld_reset(owp);
}

/**
 * @brief     Read some bytes from slave device.
 *
 * @param[in] owp       pointer to the @p onewireDriver object
 * @param[out] rxbuf    pointer to the buffer for read data
 * @param[in] rxbytes   amount of data to be received
 */
void onewireRead(onewireDriver *owp, uint8_t *rxbuf, size_t rxbytes) {

  osalDbgCheck((NULL != owp) && (NULL != rxbuf));
  osalDbgCheck((rxbytes > 0) && (rxbytes <= ONEWIRE_MAX_TRANSACTION_LEN));
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");

  /* Buffer zeroing. This is important because of driver collects
     bits using |= operation.*/
  memset(rxbuf, 0, rxbytes);

  owp->reg.bit = 0;
  owp->reg.final_timeslot = false;
  owp->rxbuf = rxbuf;
  owp->reg.bytes = rxbytes;

  onewire_lld_read(owp, ow_read_bit_cb);
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
void onewireWrite(onewireDriver *owp, const uint8_t *txbuf,
                  size_t txbytes, systime_t pullup_time) {

  osalDbgCheck((NULL != owp) && (NULL != txbuf));
  osalDbgCheck((txbytes > 0) && (txbytes <= ONEWIRE_MAX_TRANSACTION_LEN));
  osalDbgAssert(owp->reg.state == ONEWIRE_READY, "Invalid state");
#if !ONEWIRE_USE_STRONG_PULLUP
  osalDbgAssert(0 == pullup_time,
      "Non zero time is valid only when strong pull enabled");
#endif

  owp->txbuf = txbuf;
  owp->reg.bit = 0;
  owp->reg.final_timeslot = false;
  owp->reg.bytes = txbytes;

#if ONEWIRE_USE_STRONG_PULLUP
  if (pullup_time > 0) {
    owp->reg.state = ONEWIRE_PULL_UP;
    owp->reg.need_pullup = true;
  }
#endif

  onewire_lld_write(owp);

#if ONEWIRE_USE_STRONG_PULLUP
  if (pullup_time > 0) {
    osalThreadSleep(pullup_time);
    owp->config->pullup_release();
    owp->reg.state = ONEWIRE_READY;
  }
#endif
}

#if ONEWIRE_USE_SEARCH_ROM
/**
 * @brief   Performs tree search on bus.
 * @note    This function does internal 1-wire reset calls every search
 *          iteration.
 *
 * @param[in] owp         pointer to a @p OWDriver object
 * @param[out] result     pointer to buffer for discovered ROMs
 * @param[in] max_rom_cnt buffer size in ROMs count for overflow prevention
 *
 * @return              Count of discovered ROMs. May be more than max_rom_cnt.
 * @retval 0            no ROMs found or communication error occurred.
 */
size_t onewireSearchRom(onewireDriver *owp, uint8_t *result,
                        size_t max_rom_cnt) {
  uint8_t cmd;

  osalDbgCheck(NULL != owp);
  osalDbgAssert(ONEWIRE_READY == owp->reg.state, "Invalid state");
  osalDbgCheck((max_rom_cnt <= 256) && (max_rom_cnt > 0));

  cmd = ONEWIRE_CMD_SEARCH_ROM;

  search_clean_start(&owp->search_rom);

  do {
    /* every search must be started from reset pulse */
    if (false == onewireReset(owp))
      return 0;

    /* initialize buffer to store result */
    if (owp->search_rom.reg.devices_found >= max_rom_cnt)
      owp->search_rom.retbuf = result + 8*(max_rom_cnt-1);
    else
      owp->search_rom.retbuf = result + 8*owp->search_rom.reg.devices_found;
    memset(owp->search_rom.retbuf, 0, 8);

    /* clean iteration state */
    search_clean_iteration(&owp->search_rom);

    /**/
    onewireWrite(&OWD1, &cmd, 1, 0);

    onewire_lld_read(owp, ow_search_rom_cb);

    if (ONEWIRE_SEARCH_ROM_ERROR != owp->search_rom.reg.result) {
      /* check CRC and return 0 (0 == error) if mismatch */
      if (owp->search_rom.retbuf[7] != onewireCRC(owp->search_rom.retbuf, 7))
        return 0;
      /* store cached result for usage in next iteration */
      memcpy(owp->search_rom.prev_path, owp->search_rom.retbuf, 8);
    }
  }
  while (ONEWIRE_SEARCH_ROM_SUCCESS == owp->search_rom.reg.result);

  /**/
  if (ONEWIRE_SEARCH_ROM_ERROR == owp->search_rom.reg.result)
    return 0;
  else
    return owp->search_rom.reg.devices_found;
}
#endif /* ONEWIRE_USE_SEARCH_ROM */

/*
 * Include test code (if enabled).
 */
#if ONEWIRE_SYNTH_SEARCH_TEST
#include "synth_searchrom.c"
#endif

#endif /* HAL_USE_ONEWIRE */

/** @} */
