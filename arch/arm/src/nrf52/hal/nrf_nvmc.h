/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NRF_NVMC_H__
#define NRF_NVMC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @defgroup nrf_nvmc_hal NVMC HAL
 * @{
 * @ingroup nrf_nvmc
 * @brief   Hardware access layer for managing the Non-Volatile Memory Controller (NVMC) peripheral.
 *
 * This driver allows writing to the non-volatile memory (NVM) regions
 * of the chip. In order to write to NVM the controller must be powered
 * on and the relevant page must be erased.
 *
 */


/**
 * @brief Erase a page in flash. This is required before writing to any
 * address in the page.
 *
 * @param address Start address of the page.
 */
void nrf_nvmc_page_erase(uint32_t address);


/**
 * @brief Write a single byte to flash.
 *
 * The function reads the word containing the byte, and then
 * rewrites the entire word.
 *
 * @param address Address to write to.
 * @param value   Value to write.
 */
void nrf_nvmc_write_byte(uint32_t address, uint8_t value);


/**
 * @brief Write a 32-bit word to flash.
 * @param address Address to write to.
 * @param value   Value to write.
 */
void nrf_nvmc_write_word(uint32_t address, uint32_t value);


/**
 * @brief Write consecutive bytes to flash.
 *
 * @param address   Address to write to.
 * @param src       Pointer to data to copy from.
 * @param num_bytes Number of bytes in src to write.
 */
void nrf_nvmc_write_bytes(uint32_t  address, const uint8_t *src, uint32_t num_bytes);


/**
 * @brief Write consecutive words to flash.
 *
 * @param address   Address to write to.
 * @param src       Pointer to data to copy from.
 * @param num_words Number of bytes in src to write.
 */
void nrf_nvmc_write_words(uint32_t address, const uint32_t *src, uint32_t num_words);

/**
 * @brief enable I-Cache for Flash
 *
 * @param flag   Flag to enable or disable.
 */
void nrf_nvmc_enable_icache(bool flag);

/**
 * @brief enable profiling I-Cache for flash
 *
 * @param flag   Flag to enable or disable.
 */
void nrf_nvmc_enable_profile(bool flag);

/**
 * @brief get I-Hit for profiling
 *
 */
uint32_t nrf_nvmc_get_profiling_ihit(void);

/**
 * @brief get I-Miss for profiling
 *
 */
uint32_t nrf_nvmc_get_profiling_imiss(void);

/**
 * @brief get internal FLASH size
 *
 */
uint32_t nrf_nvmc_get_flash_size(void);

/**
 * @brief get internal RAM size
 *
 */
uint32_t nrf_nvmc_get_ram_size(void);

/**
 * get system  image code begin address
 *
 * @param  none
 * @return address
 *
 * @brief  get system image begin address
 */
uint32_t system_image_start_address(void);

/**
 * get system  image end address of read only section
 *
 * @param  none
 * @return address
 *
 */
uint32_t system_image_ro_section_end(void);

/**
 * get system  image data section size
 *
 * @param  none
 * @return address
 *
 */
uint32_t system_image_data_section_size(void);

/**
 * get device identifier 0
 *
 * @param  none
 * @return device id
 *
 */
uint32_t nrf_nvmc_read_dev_id0(void);

/**
 * get device identifier 1
 *
 * @param  none
 * @return device id
 *
 */
uint32_t nrf_nvmc_read_dev_id1(void);

#ifdef __cplusplus
}
#endif

#endif // NRF_NVMC_H__
/** @} */


