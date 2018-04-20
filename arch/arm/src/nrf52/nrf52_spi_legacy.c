/****************************************************************************
 * arch/arm/src/nrf52/nrf52_spi_legacy.c
 *
 *   Copyright (C) 2009-2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright (C) 2017 Zglue  Inc. All rights reserved.
 *           Levin Li     <zhiqiang@zglue.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "nrf.h"
#include "nrf_spi.h"
#include "nrf52_gpio.h"
#include "nrf52_spi.h"


#if defined(CONFIG_NRF52_LEGACY_SPI0) || defined(CONFIG_NRF52_LEGACY_SPI1) || defined(CONFIG_NRF52_LEGACY_SPI2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define NRF_DRV_SPI_PIN_NOT_USED  0xFF

#ifndef NRF_SPI_PIN_NOT_CONNECTED
#define NRF_SPI_PIN_NOT_CONNECTED  0xFFFFFFFF
#endif

#define NRF52_SPI0_CLK_FREQUENCY  NRF_SPI_FREQ_8M

#define NRF52_SPI1_CLK_FREQUENCY  NRF_SPI_FREQ_8M

#define NRF52_SPI2_CLK_FREQUENCY  NRF_SPI_FREQ_8M

// All interrupt flags
#define DISABLE_ALL_INT_SHORT  0xFFFFFFFF

#define NRF52_SPI_MAX_TRANSFER_LEN  0xFF
#define NRF52_SPI_ONCE_TRANSFER_LEN 0x80

/* Note:
 *    Nordic tx / rx buffer accept the max transfer data is 255 byte.
 *    You can check datasheet by 31.6.14 TXD.MAXCNT Address offset: 0x548
 *    they support double buffer feature , but now this driver not use double buffer
 *    feature
 *    another feature is DMA list ,  now we don't use it
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/**
 * @brief Single transfer descriptor structure.
 */
typedef struct
{
  uint8_t const *p_tx_buffer;  ///< Pointer to TX buffer.
  uint8_t        *p_rx_buffer; ///< Pointer to RX buffer.
  uint32_t          tx_length;   ///< TX buffer length.
  uint32_t          rx_length;   ///< RX buffer length.
} nrf_spi_xfer_desc_t;

struct legacy_spidev_s
{
  struct spi_dev_s          spidev;     /* Generic SPI device */
  unsigned int              base;       /* Base address of registers */
  uint16_t                  irqid;      /* IRQ for this device */
  uint8_t                   busid;
  sem_t                     mutex;      /* Only one thread can access at a time */
#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  xcpt_t                    isr;      /* Interrupt handler */
  sem_t                     wait;       /* Interrupt wait semaphore */
#endif
  nrf_spi_frequency_t       frequency;  /* Current SPI frequency */
  uint32_t                  mode;       /* Current SPI frequency */
  nrf_spi_bit_order_t       bit_order;  /* Current SPI frequency */
  uint8_t                   orc;        /* < Over-run character.
                                         **< This character is used when all bytes from the TX buffer are sent,
                                         *but the transfer continues due to RX. */
  nrf_spi_xfer_desc_t       xfer_desc;
  uint32_t                  transferred;
  nrf_drv_state_t           state;
  struct spi_pinmux_t       *pinmux;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t legacy_spi_setfrequency(FAR struct spi_dev_s *dev,
                                        uint32_t frequency);
void legacy_spi_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
/* SPI device operations */
static int legacy_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void legacy_spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static uint16_t legacy_spi_send(FAR struct spi_dev_s *dev, uint16_t wd);

#ifdef CONFIG_SPI_EXCHANGE
static void legacy_spi_exchange(FAR struct spi_dev_s *dev,
                                FAR const void *txbuffer, FAR void *rxbuffer,
                                size_t nwords);
#else
static void legacy_spi_sndblock(FAR struct spi_dev_s *dev,
                                FAR const void *txbuffer, size_t nwords);
static void legacy_spi_recvblock(FAR struct spi_dev_s *dev,
                                 FAR void *buffer, size_t nwords);
#endif

#ifdef CONFIG_SPI_RESET
static int  legacy_spi_reset(FAR struct spi_dev_s *dev);
#endif

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
static int  legacy_spi_interrupt_handler(int irq, void *context, FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct spi_ops_s legacy_spi_ops =
{
  .lock              = legacy_spi_lock,
  .setfrequency      = legacy_spi_setfrequency,
  .setmode           = legacy_spi_setmode,
  .setbits           = NULL,
  .select            = legacy_spi_select,
  .send              = legacy_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = legacy_spi_exchange,
#else
  .sndblock          = legacy_spi_sndblock,
  .recvblock         = legacy_spi_recvblock,
#endif
#ifdef CONFIG_SPI_RESET
  .reset             = legacy_spi_reset
#endif
};

#ifdef CONFIG_NRF52_LEGACY_SPI0
static const struct spi_pinmux_t spi0_pinmux =
{
  .cs_pin     = BOARD_SPI0_CS_PIN,
  .miso_pin   = BOARD_SPI0_MISO_PIN,
  .mosi_pin   = BOARD_SPI0_MOSI_PIN,
  .sck_pin    = BOARD_SPI0_SCL_PIN,
};

static struct legacy_spidev_s g_spi0 =
{
  .spidev     = { &legacy_spi_ops },
  .base       = NRF_SPIM0_BASE,
  .irqid      = NRF52_IRQ_SPI_TWI_0,
  .busid      = SPI_NRF52_BUS_0,
  .frequency  = NRF52_SPI0_CLK_FREQUENCY,
  .mode       = NRF_SPI_MODE_0,
  .orc        = 0xA5,
  .bit_order  = NRF_SPI_BIT_ORDER_MSB_FIRST,
#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  .isr        = legacy_spi_interrupt_handler,
#endif
  .pinmux     = (struct spi_pinmux_t *) &spi0_pinmux
};
#endif

#ifdef CONFIG_NRF52_SPI1
static const struct spi_pinmux_t spi1_pinmux =
{
  .cs_pin     = BOARD_SPI1_CS_PIN,
  .miso_pin   = BOARD_SPI1_MISO_PIN,
  .mosi_pin   = BOARD_SPI1_MOSI_PIN,
  .sck_pin    = BOARD_SPI1_SCL_PIN,
};

static struct legacy_spidev_s g_spi1 =
{
  .spidev     = { &legacy_spi_ops },
  .base       = NRF_SPIM1_BASE,
  .irqid      = NRF52_IRQ_SPI_TWI_1,
  .busid      = SPI_NRF52_BUS_1,
  .frequency  = NRF52_SPI1_CLK_FREQUENCY,
  .mode       = NRF_SPI_MODE_0,
  .orc        = 0xA5,
  .bit_order  = NRF_SPI_BIT_ORDER_MSB_FIRST,

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  .isr        = legacy_spi_interrupt_handler,
#endif
  .pinmux     = (struct spi_pinmux_t *) &spi1_pinmux
};
#endif

#ifdef CONFIG_NRF52_LEGACY_SPI2

static const struct spi_pinmux_t spi2_pinmux =
{
  .cs_pin     = BOARD_SPI2_CS_PIN,
  .miso_pin   = BOARD_SPI2_MISO_PIN,
  .mosi_pin   = BOARD_SPI2_MOSI_PIN,
  .sck_pin    = BOARD_SPI2_SCL_PIN,
};

static struct legacy_spidev_s g_spi2 =
{
  .spidev     = { &legacy_spi_ops },
  .base       = NRF_SPIM2_BASE,
  .irqid      = NRF52_IRQ_SPI2,
  .busid      = SPI_NRF52_BUS_2,
  .frequency  = NRF52_SPI2_CLK_FREQUENCY,
  .mode       = NRF_SPI_MODE_0,
  .orc        = 0xA5,
  .bit_order  = NRF_SPI_BIT_ORDER_MSB_FIRST,

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  .isr        = legacy_spi_interrupt_handler,
#endif
  .pinmux     = (struct spi_pinmux_t *) &spi2_pinmux
};
#endif

#ifdef CONFIG_NRF52_CS_CONTROL_BY_USER
static SPI_CS_CallBack g_cs_select;
#endif
/******************************************************************************
 * Private Functions
 *****************************************************************************/


/****************************************************************************
 * Name: legacy_spi_setfrequency
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static uint32_t legacy_spi_setfrequency(FAR struct spi_dev_s *dev,
                                        uint32_t frequency)
{
  struct legacy_spidev_s *priv = (struct legacy_spidev_s *) dev;
  NRF_SPI_Type *p_spi = (NRF_SPI_Type *)priv->base;
  uint32_t freq_k ;

  freq_k = frequency / 1000;

  switch (freq_k)
    {
      case 125:
        priv->frequency = NRF_SPI_FREQ_125K;
        break;
      case 250:
        priv->frequency = NRF_SPI_FREQ_250K;
        break;
      case 500:
        priv->frequency = NRF_SPI_FREQ_500K;
        break;
      case 1000:
        priv->frequency = NRF_SPI_FREQ_1M;
        break;
      case 2000:
        priv->frequency = NRF_SPI_FREQ_2M;
        break;
      case 4000:
        priv->frequency = NRF_SPI_FREQ_4M;
        break;
      case 8000:
        priv->frequency = NRF_SPI_FREQ_8M;
        break;
      default:
        spierr("Legacy: Invalid Parameter for SPI frequency , %d\n", frequency);
        return -EINVAL;
        break;
    }
  nrf_spi_frequency_set(p_spi, (nrf_spi_frequency_t)priv->frequency);

  return frequency;
}

/****************************************************************************
 * Name: legacy_spi_select
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
void legacy_spi_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  struct legacy_spidev_s *priv = (struct legacy_spidev_s *) dev;

#ifdef CONFIG_NRF52_CS_CONTROL_BY_USER

  if (NULL == g_cs_select)
    {
      spierr("Please register customized  CS select API.\n");
      return;
    }

  g_cs_select(priv->busid, devid, selected);

#else

  struct spi_pinmux_t *pinmux = priv->pinmux;

  if (selected == true)
    {
      if (pinmux->cs_pin != NRF52_INVALID_GPIO_PIN)
        {
          nrf52_gpio_write(pinmux->cs_pin, true);
        }
    }
  else
    {
      if (pinmux->cs_pin != NRF52_INVALID_GPIO_PIN)
        {
          nrf52_gpio_write(pinmux->cs_pin, false);
        }
    }
#endif
}

/*****************************************************************************
 * Name: legacy_spi_setmode
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void legacy_spi_setmode(FAR struct spi_dev_s *dev,
                               enum spi_mode_e mode)
{
  struct legacy_spidev_s *priv = (struct legacy_spidev_s *) dev;
  NRF_SPI_Type *p_spi = (NRF_SPI_Type *)priv->base;
  uint32_t config = p_spi->CONFIG;
  switch (mode)
    {
      case SPIDEV_MODE0:
        config |= (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Leading    << SPIM_CONFIG_CPHA_Pos);
        break;
      case SPIDEV_MODE1:
        config |= (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Trailing   << SPIM_CONFIG_CPHA_Pos);
        break;
      case SPIDEV_MODE2:
        config |= (SPIM_CONFIG_CPOL_ActiveLow  << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Leading    << SPIM_CONFIG_CPHA_Pos);
        break;
      case SPIDEV_MODE3:
        config |= (SPIM_CONFIG_CPOL_ActiveLow  << SPIM_CONFIG_CPOL_Pos) |
                  (SPIM_CONFIG_CPHA_Trailing   << SPIM_CONFIG_CPHA_Pos);
        break;
    }
  p_spi->CONFIG = config;
}

/******************************************************************************
 * Name: legacy_spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

static int legacy_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct legacy_spidev_s *priv = (FAR struct legacy_spidev_s *)dev;

  if (lock == true)
    {
      /* Take the semaphore (perhaps waiting) */
      while (sem_wait(&priv->mutex) != 0)
        {
          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->mutex);
    }
  return OK;
}


/* This function is called from IRQ handler or, in blocking mode, directly
 * from the 'nrf_drv_spi_transfer' function.
 * It returns true as long as the transfer should be continued, 
 * otherwise (whenthere is nothing more to send/receive) it returns false.
 ***************************************************************************/
static bool transfer_byte(FAR struct legacy_spidev_s *priv, NRF_SPI_Type *p_spi)
{
  /* Read the data byte received in this transfer and store it in RX buffer,
   * if needed.
   ************************************************************************/
  volatile uint8_t rx_data = nrf_spi_rxd_get(p_spi);
  if (priv->transferred < priv->xfer_desc.rx_length)
    {
      priv->xfer_desc.p_rx_buffer[priv->transferred] = rx_data;
    }

  ++priv->transferred;

  /* Check if there are more bytes to send or receive and write proper data
   * byte (next one from TX buffer or over-run character) to the TXD register
   * when needed.
   * NOTE - we've already used 'p_cb->bytes_transferred + 1' bytes from our
   *        buffers, because we take advantage of double buffering of TXD
   *        register (so in effect one byte is still being transmitted now);
   *        see how the transfer is started in the 'nrf_drv_spi_transfer'
   *        function.
   *************************************************************************/
  uint16_t bytes_used = priv->transferred + 1;

  if (bytes_used < priv->xfer_desc.tx_length)
    {
      nrf_spi_txd_set(p_spi, priv->xfer_desc.p_tx_buffer[bytes_used]);
      return true;
    }
  else if (bytes_used < priv->xfer_desc.rx_length)
    {
      nrf_spi_txd_set(p_spi, priv->orc);
      return true;
    }

  return (priv->transferred < priv->xfer_desc.tx_length ||
          priv->transferred < priv->xfer_desc.rx_length);
}

/******************************************************************************
 * Name: nrf_spi_xfer
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ******************************************************************************/
static ret_code_t nrf_spi_xfer(FAR struct legacy_spidev_s *priv,
                               NRF_SPI_Type                 *p_spi,
                               nrf_spi_xfer_desc_t const *p_xfer_desc)
{
  ret_code_t err_code;

  nrf_spi_int_disable(p_spi, NRF_SPI_INT_READY_MASK);
  nrf_spi_event_clear(p_spi, NRF_SPI_EVENT_READY);


  /* Start the transfer by writing some byte to the TXD register;
   * if TX buffer is not empty, take the first byte from this buffer,
   * otherwise - use over-run character.
   ******************************************************************/

  nrf_spi_txd_set(p_spi,
                  (p_xfer_desc->tx_length > 0 ?  p_xfer_desc->p_tx_buffer[0] : priv->orc));

  /* TXD register is double buffered, so next byte to be transmitted can
   * be written immediately, if needed, i.e. if TX or RX transfer is to
   * be more that 1 byte long. Again - if there is something more in TX
   * buffer send it, otherwise use over-run character.
   *******************************************************************/

  if (p_xfer_desc->tx_length > 1)
    {
      nrf_spi_txd_set(p_spi, p_xfer_desc->p_tx_buffer[1]);
    }
  else if (p_xfer_desc->rx_length > 1)
    {
      nrf_spi_txd_set(p_spi, priv->orc);
    }

#ifdef CONFIG_NRF52_SPI_INTERRUPTS

  /* enable interrupt and start task */

  nrf_spi_int_enable(p_spi, NRF_SPI_INT_READY_MASK);

  /* if there is interrupt , just wait the semephore
   * potential issue: if interrupt missing , no post action
   */

  sem_wait(&priv->wait);

#else

  do
    {
      while (!nrf_spi_event_check(p_spi, NRF_SPI_EVENT_READY)) {}
      nrf_spi_event_clear(p_spi, NRF_SPI_EVENT_READY);
      spiinfo("SPI: Event: NRF_SPI_EVENT_READY.");
    }
  while (transfer_byte(priv, p_spi));

#endif

  err_code = OK;
  return err_code;
}

/******************************************************************************
 * Name: nrf_drv_spi_transfer
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 *****************************************************************************/
static ret_code_t nrf_legacy_spi_transfer(FAR struct legacy_spidev_s *priv,
                                          uint8_t const *p_tx_buffer,
                                          uint32_t        tx_buffer_length,
                                          uint8_t        *p_rx_buffer,
                                          uint32_t        rx_buffer_length)
{
  priv->xfer_desc.p_tx_buffer = p_tx_buffer;
  priv->xfer_desc.p_rx_buffer = p_rx_buffer;
  priv->xfer_desc.tx_length   = tx_buffer_length;
  priv->xfer_desc.rx_length   = rx_buffer_length;
  priv->transferred = 0;
  return nrf_spi_xfer(priv, (NRF_SPI_Type *)priv->base, &priv->xfer_desc);
}

#ifdef CONFIG_NRF52_SPI_INTERRUPTS

/****************************************************************************
 * Name: legacy_spi_interrupt
 *
 * Description:
 *   The SPI Interrupt Handler
 *
 ****************************************************************************/
static int legacy_spi_interrupt_handler(int irq, void *context, FAR void *arg)
{
  struct legacy_spidev_s *dev = (struct legacy_spidev_s *)arg;

  NRF_SPI_Type *p_spi = (NRF_SPI_Type *)dev->base;

  if (nrf_spi_event_check(p_spi, NRF_SPI_EVENT_READY))
    {
      irqinfo("SPI: Ready Event.\n");

      nrf_spi_event_clear(p_spi, NRF_SPI_EVENT_READY);

      if (!transfer_byte(dev, p_spi))
        {
          /* Signal that the transfer is complete */

          sem_post(&(dev->wait));
        }
    }

  return OK;
}

#endif

/******************************************************************************
 * Name: legacy_spi_reset
 *
 * Description:
 *   Perform an SPI bus reset in an attempt to break loose stuck SPI devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 *****************************************************************************/

#ifdef CONFIG_SPI_RESET
static int legacy_spi_reset(struct spi_dev_s *dev)
{
  return OK;
}
#endif /* CONFIG_SPI_RESET */

/*****************************************************************************
 * Public Functions
 ****************************************************************************/
static int32_t nrf_legacy_spi_init(struct legacy_spidev_s *dev)
{
  uint32_t mosi_pin;
  uint32_t miso_pin;
  ret_code_t err_code;
  NRF_SPI_Type *p_spi = (NRF_SPI_Type *)dev->base;
  struct spi_pinmux_t *p_pinmux = dev->pinmux;

  ASSERT(dev);

  if (dev->state != NRF_DRV_STATE_UNINITIALIZED)
    {
      err_code = EINVAL;
      return err_code;
    }

  /* Configure pins used by the peripheral:
   * - SCK - output with initial value corresponding with the SPI mode used:
   *   0 - for modes 0 and 1 (CPOL = 0), 1 - for modes 2 and 3 (CPOL = 1);
   *   according to the reference manual guidelines this pin and its input
   *   buffer must always be connected for the SPI to work.
   */

  if (dev->mode <= NRF_SPI_MODE_1)
    {
      nrf52_gpio_write(p_pinmux->sck_pin, false);
    }
  else
    {
      nrf52_gpio_write(p_pinmux->sck_pin, true);
    }

  nrf52_gpio_config(p_pinmux->sck_pin);

  /* - MOSI (optional) - output with initial value 0 */

  if (p_pinmux->mosi_pin != NRF52_INVALID_GPIO_PIN)
    {
      mosi_pin = p_pinmux->mosi_pin;
      nrf52_gpio_write(mosi_pin, false);
      nrf52_gpio_config(mosi_pin);
    }
  else
    {
      mosi_pin = NRF_SPI_PIN_NOT_CONNECTED;
    }

  /* - MISO (optional) - input */

  if (p_pinmux->miso_pin != NRF52_INVALID_GPIO_PIN)
    {
      miso_pin = p_pinmux->miso_pin;
      nrf_gpio_cfg_input(miso_pin, GPIO_PIN_CNF_PULL_Pulldown);
    }
  else
    {
      miso_pin = NRF52_INVALID_GPIO_PIN;
    }

#ifndef CONFIG_NRF52_CS_CONTROL_BY_USER
  /* - Slave Select (optional) - output with initial value 1 (inactive). */

  if (p_pinmux->cs_pin != NRF52_INVALID_GPIO_PIN)
    {
      nrf52_gpio_write(p_pinmux->cs_pin, true);
      nrf52_gpio_config(p_pinmux->cs_pin);
    }
#endif

  nrf_spi_pins_set(p_spi, p_pinmux->sck_pin, mosi_pin, miso_pin);
  nrf_spi_frequency_set(p_spi, (nrf_spi_frequency_t)dev->frequency);
  nrf_spi_configure(p_spi,
                    (nrf_spi_mode_t)dev->mode,
                    (nrf_spi_bit_order_t)dev->bit_order);

  dev->state = NRF_DRV_STATE_INITIALIZED;

  return OK;
}

#ifdef CONFIG_NRF52_CS_CONTROL_BY_USER
FAR int nrf52_spibus_register(SPI_CS_CallBack cs_select)
{
  g_cs_select = cs_select;
  return OK;
}
#endif

/****************************************************************************
 * Name: legacy_spibus_initialize
 *
 * Description:
 *   Initialise an SPI device
 *
 ****************************************************************************/

struct spi_dev_s *legacy_spibus_initialize(int port)
{
  struct legacy_spidev_s *priv = NULL;
  if (port > 2)
    {
      spierr("ERROR: NRF52 SPI Only supports ports 0 and 1 & 2\n");
      return NULL;
    }

  irqstate_t flags;
  int32_t err_code;

#ifdef CONFIG_NRF52_LEGACY_SPI0
  if (port == 0)
    {
      priv = &g_spi0;
    }
#endif

#ifdef CONFIG_NRF52_LEGACY_SPI1
  if (port == 1)
    {
      priv = &g_spi1;
    }
#endif

#ifdef CONFIG_NRF52_LEGACY_SPI2
  if (port == 2)
    {
      priv = &g_spi2;
    }
#endif

  if (NULL == priv)
    {
      spierr("Wrong Parameter or Config Error, Please Check !!!\n");
      return NULL;
    }

  err_code = nrf_legacy_spi_init(priv);

  if (err_code != OK)
    {
      return NULL;
    }

  /* Initialize semaphores */

  sem_init(&priv->mutex, 0, 1);

#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  sem_init(&priv->wait, 0, 0);

  /* The wait semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
  */
  sem_setprotocol(&priv->wait, SEM_PRIO_NONE);
#endif

  flags = enter_critical_section();

#ifdef CONFIG_NRF52_SPI_INTERRUPTS

  /* Attach Interrupt Handler */
  irq_attach(priv->irqid, priv->isr, priv);

  /* Enable Interrupt Handler */
  up_enable_irq(priv->irqid);
#endif

  leave_critical_section(flags);

  /* Enable SPI */

  nrf_spi_enable((NRF_SPI_Type *)priv->base);

  return &(priv->spidev);
}



/****************************************************************************
 * Name: legacy_spibus_uninitialize
 *
 * Description:
 *   Uninitialise an SPI device
 *
 ****************************************************************************/

int legacy_spibus_uninitialize(FAR struct spi_dev_s *dev)
{
  struct legacy_spidev_s *priv = (struct legacy_spidev_s *) dev;
  irqstate_t flags;

  ASSERT(priv->state != NRF_DRV_STATE_UNINITIALIZED);

  /* Disable SPI */
  nrf_spi_disable((NRF_SPI_Type *)priv->base);

  /* Reset data structures */

  sem_destroy(&priv->mutex);
#ifdef CONFIG_NRF52_SPI_INTERRUPTS
  sem_destroy(&priv->wait);
#endif

  flags = enter_critical_section();

  /* Disable interrupts */

  up_disable_irq(priv->irqid);

  /* Detach Interrupt Handler */

  irq_detach(priv->irqid);

  leave_critical_section(flags);

  priv->state = NRF_DRV_STATE_UNINITIALIZED;
  return OK;
}

/****************************************************************************
 * Name: legacy_spi_send
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static uint16_t legacy_spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct legacy_spidev_s *priv = (struct legacy_spidev_s *)dev;
  uint8_t response = 0;
  uint8_t cmd;

  cmd = (uint8_t)wd;

  response = 0;

  /* issue here:
   *    spi_send should return the response data , but there is bug
   *    if use nrf_drv_spi_transfer(priv, &cmd, 1, &response, 1) to get response
   *    nordic will send out two command, bus data is wrong.
   *    Should get workaround in further
   */

  nrf_legacy_spi_transfer(priv, &cmd, 1, &response, 1);

  spiinfo("Legacy: Cmd: %02#x. Response: %02#x\n", cmd, response);

  return (uint16_t)response;
}

#ifdef CONFIG_SPI_EXCHANGE
/************************************************************************************
 * Name: legacy_spi_exchange
 *
 * Description:
 *   Release exclusive access
 *
 ************************************************************************************/
static void legacy_spi_exchange(FAR struct spi_dev_s *dev,
                                      FAR const void *txbuffer,
                                      FAR void *rxbuffer, size_t nwords)
{
  FAR struct legacy_spidev_s *priv = (struct legacy_spidev_s *)dev;

  spiinfo("Legacy: Exchange: size %d.\n", nwords);

  nrf_legacy_spi_transfer(priv, (uint8_t const *)txbuffer,
                         nwords, (uint8_t *)rxbuffer, nwords);

}

#else
/****************************************************************************
 * Name: legacy_spi_sndblock
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void legacy_spi_sndblock(FAR struct spi_dev_s *dev,
                                      FAR const void *txbuffer, size_t nwords)
{
  FAR struct legacy_spidev_s *priv = (struct legacy_spidev_s *)dev;

  spiinfo("Legacy: Sndblock: size %d.\n", nwords);

  nrf_legacy_spi_transfer(priv, (uint8_t const *)txbuffer, nwords, NULL, 0);

}

/****************************************************************************
 * Name: legacy_spi_recvblock
 *
 * Description:
 *   Set the frequency for the next transfer
 *
 ****************************************************************************/
static void legacy_spi_recvblock(FAR struct spi_dev_s *dev,
                                        FAR void *rxbuffer, size_t nwords)
{
  FAR struct legacy_spidev_s *priv = (struct legacy_spidev_s *)dev;

  spiinfo("Recvblock: size %d.\n", nwords);

  nrf_legacy_spi_transfer(priv, NULL, 0, (uint8_t const *)rxbuffer, nwords);

}

#endif

#endif /*CONFIG_NRF52_LEGACY_SPI0 CONFIG_NRF52_LEGACY_SPI1 CONFIG_NRF52_LEGACY_SPI2 */

