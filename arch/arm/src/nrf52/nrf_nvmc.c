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
/**
 *@file
 *@brief NMVC driver implementation
 */

#include <nuttx/config.h>
#include <stdbool.h>
#include "cache.h"
#include "nrf.h"
#include "nrf_nvmc.h"
#include "nrf52_waste.h"

static inline void wait_for_flash_ready(void)
{
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {;}
}

static inline void nrf_mem_barrier(void)
{
  ARM_ISB();
  ARM_DSB();
}

void nrf_nvmc_enable_icache(bool flag)
{
  uint32_t value = NRF_NVMC->ICACHECNF;

  if (flag)
    {
      value |= NVMC_ICACHECNF_CACHEEN_Msk;
    }
  else
    {
      value &= ~NVMC_ICACHECNF_CACHEEN_Msk;
    }

  NRF_NVMC->ICACHECNF = value;
}

void nrf_nvmc_enable_profile(bool flag)
{
  uint32_t value = NRF_NVMC->ICACHECNF;

  if (flag)
    {
      value |= NVMC_ICACHECNF_CACHEPROFEN_Msk;
    }
  else
    {
      value &= ~NVMC_ICACHECNF_CACHEPROFEN_Msk;
    }

  NRF_NVMC->ICACHECNF = value;

}

uint32_t nrf_nvmc_get_profiling_ihit(void)
{
  return NRF_NVMC->IHIT;
}

uint32_t nrf_nvmc_get_profiling_imiss(void)
{
  return NRF_NVMC->IMISS;
}

uint32_t nrf_nvmc_get_flash_size(void)
{
  return NRF_FICR->INFO.FLASH * 1024;
}

uint32_t nrf_nvmc_get_ram_size(void)
{
  return NRF_FICR->INFO.RAM * 1024;
}

uint32_t nrf_nvmc_read_dev_id0(void)
{
  return NRF_FICR->DEVICEID[0];
}

uint32_t nrf_nvmc_read_dev_id1(void)
{
  return NRF_FICR->DEVICEID[1];
}

uint32_t system_image_start_address(void)
{
  extern uint32_t _stext;
  return (uint32_t)&_stext;
}

uint32_t system_image_ro_section_end(void)
{
  extern uint32_t _eronly;
  return (uint32_t)&_eronly;
}

uint32_t system_image_data_section_size(void)
{
  extern uint32_t _edata;
  extern uint32_t _sdata;
  uint32_t data_size;
  uint32_t start;
  uint32_t end;
  start = (uint32_t)&_sdata;
  end = (uint32_t)&_edata;
  data_size = end - start;
  return data_size;
}

void nrf_nvmc_page_erase(uint32_t address)
{
  // Enable erase.
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
  nrf_mem_barrier();

  // Erase the page
  NRF_NVMC->ERASEPAGE = address;
  wait_for_flash_ready();

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
  nrf_mem_barrier();

}


void nrf_nvmc_write_byte(uint32_t address, uint8_t value)
{
  uint32_t byte_shift = address & (uint32_t)0x03;
  uint32_t address32 = address & ~byte_shift; // Address to the word this byte is in.
  uint32_t value32 = (*(uint32_t *)address32 & ~((uint32_t)0xFF << (byte_shift << (uint32_t)3)));
  value32 = value32 + ((uint32_t)value << (byte_shift << 3));

  // Enable write.
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  nrf_mem_barrier();

  *(uint32_t *)address32 = value32;
  wait_for_flash_ready();

  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  nrf_mem_barrier();

}

void nrf_nvmc_write_word(uint32_t address, uint32_t value)
{
  // Enable write.
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
  nrf_mem_barrier();

  *(uint32_t *)address = value;
  wait_for_flash_ready();

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
  nrf_mem_barrier();

}

void nrf_nvmc_write_bytes(uint32_t address, const uint8_t *src, uint32_t num_bytes)
{
  uint32_t i;
  for (i = 0; i < num_bytes; i++)
    {
      nrf_nvmc_write_byte(address + i, src[i]);
    }
}

void nrf_nvmc_write_words(uint32_t address, const uint32_t *src, uint32_t num_words)
{
  uint32_t i;

  // Enable write.
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
  nrf_mem_barrier();

  for (i = 0; i < num_words; i++)
    {
      ((uint32_t *)address)[i] = src[i];
      wait_for_flash_ready();
    }

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
  nrf_mem_barrier();

}

