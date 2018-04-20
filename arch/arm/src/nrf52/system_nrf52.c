/* Copyright (c) 2012 ARM LIMITED
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   * Neither the name of ARM nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without specific
 *     prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "cache.h"
#include "up_internal.h"

/*lint ++flb "Enter library region" */

#define __SYSTEM_CLOCK_64M      (64000000UL)

static bool errata_16(void);
static bool errata_31(void);
static bool errata_32(void);
static bool errata_36(void);
static bool errata_37(void);
static bool errata_57(void);
static bool errata_66(void);
static bool errata_108(void);


/* CoreDebug & SCB is only for errata correction */
typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

typedef struct
{
  __IOM uint32_t DHCSR;                  /*!< Offset: 0x000 (R/W)  Debug Halting Control and Status Register */
  __OM  uint32_t DCRSR;                  /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register */
  __IOM uint32_t DCRDR;                  /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register */
  __IOM uint32_t DEMCR;                  /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register */
} CoreDebug_Type;

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address */

/* Debug Exception and Monitor Control Register Definitions */
#define CoreDebug_DEMCR_TRCENA_Pos         24U                                            /*!< CoreDebug DEMCR: TRCENA Position */
#define CoreDebug_DEMCR_TRCENA_Msk         (1UL << CoreDebug_DEMCR_TRCENA_Pos)            /*!< CoreDebug DEMCR: TRCENA Mask */

#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct */
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */

#if defined ( __CC_ARM )
uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_64M;
#elif defined ( __ICCARM__ )
__root uint32_t SystemCoreClock = __SYSTEM_CLOCK_64M;
#elif defined ( __GNUC__ )
uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_64M;
#endif

void SystemCoreClockUpdate(void)
{
  SystemCoreClock = __SYSTEM_CLOCK_64M;
}

void SystemInit(void)
{
  /* Workaround for Errata 16 "System: RAM may be corrupt on wakeup from CPU IDLE" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/ */
  if (errata_16())
    {
      *(volatile uint32_t *)0x4007C074 = 3131961357ul;
    }

  /* Workaround for Errata 31 "CLOCK: Calibration values are not correctly loaded from FICR at reset" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/ */
  if (errata_31())
    {
      *(volatile uint32_t *)0x4000053C = ((*(volatile uint32_t *)0x10000244) & 0x0000E000) >> 13;
    }

  /* Workaround for Errata 32 "DIF: Debug session automatically enables TracePort pins" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/ */
  if (errata_32())
    {
      CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
    }

  /* Workaround for Errata 36 "CLOCK: Some registers are not reset when expected" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/  */
  if (errata_36())
    {
      NRF_CLOCK->EVENTS_DONE = 0;
      NRF_CLOCK->EVENTS_CTTO = 0;
      NRF_CLOCK->CTIV = 0;
    }

  /* Workaround for Errata 37 "RADIO: Encryption engine is slow by default" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/  */
  if (errata_37())
    {
      *(volatile uint32_t *)0x400005A0 = 0x3;
    }

  /* Workaround for Errata 57 "NFCT: NFC Modulation amplitude" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/  */
  if (errata_57())
    {
      *(volatile uint32_t *)0x40005610 = 0x00000005;
      *(volatile uint32_t *)0x40005688 = 0x00000001;
      *(volatile uint32_t *)0x40005618 = 0x00000000;
      *(volatile uint32_t *)0x40005614 = 0x0000003F;
    }

  /* Workaround for Errata 66 "TEMP: Linearity specification not met with default settings" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/  */
  if (errata_66())
    {
      NRF_TEMP->A0 = NRF_FICR->TEMP.A0;
      NRF_TEMP->A1 = NRF_FICR->TEMP.A1;
      NRF_TEMP->A2 = NRF_FICR->TEMP.A2;
      NRF_TEMP->A3 = NRF_FICR->TEMP.A3;
      NRF_TEMP->A4 = NRF_FICR->TEMP.A4;
      NRF_TEMP->A5 = NRF_FICR->TEMP.A5;
      //NRF_TEMP->B0 = NRF_FICR->TEMP.B0;  //XXX comment out to make compile not fail
      NRF_TEMP->B1 = NRF_FICR->TEMP.B1;
      NRF_TEMP->B2 = NRF_FICR->TEMP.B2;
      NRF_TEMP->B3 = NRF_FICR->TEMP.B3;
      NRF_TEMP->B4 = NRF_FICR->TEMP.B4;
      NRF_TEMP->B5 = NRF_FICR->TEMP.B5;
      NRF_TEMP->T0 = NRF_FICR->TEMP.T0;
      NRF_TEMP->T1 = NRF_FICR->TEMP.T1;
      NRF_TEMP->T2 = NRF_FICR->TEMP.T2;
      NRF_TEMP->T3 = NRF_FICR->TEMP.T3;
      NRF_TEMP->T4 = NRF_FICR->TEMP.T4;
    }

  /* Workaround for Errata 108 "RAM: RAM content cannot be trusted upon waking up from System ON Idle or System OFF mode" found at the Errata document
     for your device located at https://infocenter.nordicsemi.com/  */
  if (errata_108())
    {
      *(volatile uint32_t *)0x40000EE4 = *(volatile uint32_t *)0x10000258 & 0x0000004F;
    }

  /* Enable the FPU if the compiler used floating point unit instructions. __FPU_USED is a MACRO defined by the
   * compiler. Since the FPU consumes energy, remember to disable FPU use in the compiler if floating point unit
   * operations are not used in your code. */
#if defined(CONFIG_ARCH_FPU) && (CONFIG_ARCH_FPU == 1)
  SCB->CPACR |= (3UL << 20) | (3UL << 22);
  ARM_DSB();
  ARM_ISB();
#endif

  /* Configure NFCT pins as GPIOs if NFCT is not to be used in your code. If CONFIG_NFCT_PINS_AS_GPIOS is not defined,
     two GPIOs (see Product Specification to see which ones) will be reserved for NFC and will not be available as
     normal GPIOs. */
#if defined (CONFIG_NFCT_PINS_AS_GPIOS)
  if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos))
    {
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      up_systemreset();
    }
#endif

  /* Configure GPIO pads as pPin Reset pin if Pin Reset capabilities desired. If CONFIG_GPIO_AS_PINRESET is not
    defined, pin reset will not be available. One GPIO (see Product Specification to see which one) will then be
    reserved for PinReset and not available as normal GPIO. */
#if defined (CONFIG_GPIO_AS_PINRESET)
  if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
      ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)))
    {
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_UICR->PSELRESET[0] = 21;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_UICR->PSELRESET[1] = 21;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
      up_systemreset();
    }
#endif

  /* Enable SWO trace functionality. If ENABLE_SWO is not defined, SWO pin will be used as GPIO (see Product
     Specification to see which one). */
#if defined (ENABLE_SWO)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  NRF_CLOCK->TRACECONFIG |= CLOCK_TRACECONFIG_TRACEMUX_Serial << CLOCK_TRACECONFIG_TRACEMUX_Pos;
  NRF_P0->PIN_CNF[18] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
#endif

  /* Enable Trace functionality. If ENABLE_TRACE is not defined, TRACE pins will be used as GPIOs (see Product
     Specification to see which ones). */
#if defined (ENABLE_TRACE)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  NRF_CLOCK->TRACECONFIG |= CLOCK_TRACECONFIG_TRACEMUX_Parallel << CLOCK_TRACECONFIG_TRACEMUX_Pos;
  NRF_P0->PIN_CNF[14] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[15] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[16] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[18] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[20] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
#endif

//  SystemCoreClockUpdate();
}


static bool errata_16(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30)
        {
          return true;
        }
    }

  return false;
}

static bool errata_31(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30)
        {
          return true;
        }
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40)
        {
          return true;
        }
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50)
        {
          return true;
        }
    }

  return false;
}

static bool errata_32(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30)
        {
          return true;
        }
    }

  return false;
}

static bool errata_36(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30)
        {
          return true;
        }
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40)
        {
          return true;
        }
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50)
        {
          return true;
        }
    }

  return false;
}

static bool errata_37(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30)
        {
          return true;
        }
    }

  return false;
}

static bool errata_57(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30)
        {
          return true;
        }
    }

  return false;
}

static bool errata_66(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50)
        {
          return true;
        }
    }

  return false;
}


static bool errata_108(void)
{
  if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0))
    {
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30)
        {
          return true;
        }
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40)
        {
          return true;
        }
      if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50)
        {
          return true;
        }
    }

  return false;
}


/*lint --flb "Leave library region" */
