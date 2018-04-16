/****************************************************************************
  * arch/arm/src/nrf52/nrf52_wdg.c
  *
  *   Copyright (C) 2012, 2017 Zglue Inc. All rights reserved.
  *   Author: Levin Li <zhiqiang@zglue.com>
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
  ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "nrf.h"
#include "nrf_wdt.h"
#include "nrf52_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_NRF52_WDG)

/****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/
/* Clocking *****************************************************************/
/* The watchdog is alway running under 32.768KHz:
  *
  *  Fmin = Flsi / 32768
  *
  * So the maximum delay (in milliseconds) is then:
  *
  *   1000 * WDG_MAX_VALUE / 32768
  *
  * For example, if Flsi = 30Khz (the nominal, uncalibrated value), then the
  * maximum delay is:
  *
  *   Fmin = 0.03 ms
  *   Fmax = 131702 Sec
  */
#define WDG_FMIN       (1)
#define WDG_MAXTIMEOUT (1000 * (UINT32_MAX / 32768))

/* Configuration ************************************************************/

// <o> WDT_CONFIG_BEHAVIOUR  - WDT behavior in CPU SLEEP or HALT mode

// <1=> Run in SLEEP, Pause in HALT
// <8=> Pause in SLEEP, Run in HALT
// <9=> Run in SLEEP and HALT
// <0=> Pause in SLEEP and HALT

#define WDT_CONFIG_BEHAVIOUR NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT

#define WDT_CONFIG_IRQ_PRIORITY 7

/****************************************************************************
  * Private Types
  ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
  * driver state structure.  This structure must be cast-compatible with the
  * well-known watchdog_lowerhalf_s structure.
  */

struct nrf52_wdg_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  uint32_t timeout;   /* The (actual) timeout */
  uint32_t lastreset; /* The last reset time */
  bool     started;   /* true: The watchdog timer has been started */
  uint16_t reload;    /* Timer reload value */
  uint16_t behaviour; /* watchdog behaviour under sleep and halt of CPU*/
};

/****************************************************************************
  * Private Function Prototypes
  ****************************************************************************/
/* Register operations ******************************************************/


/* "Lower half" driver methods **********************************************/

static int      nrf52_start(FAR struct watchdog_lowerhalf_s *lower);
static int      nrf52_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      nrf52_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      nrf52_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                FAR struct watchdog_status_s *status);
static int      nrf52_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout);

/****************************************************************************
  * Private Data
  ****************************************************************************/
/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = nrf52_start,
  .stop       = nrf52_stop,
  .keepalive  = nrf52_keepalive,
  .getstatus  = nrf52_getstatus,
  .settimeout = nrf52_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct nrf52_wdg_lowerhalf_s g_wdgdev;

/****************************************************************************
  * Private Functions
  ****************************************************************************/

/****************************************************************************
  * Name: nrf52_start
  *
  * Description:
  *   Start the watchdog timer, resetting the time to the current timeout,
  *
  * Input Parameters:
  *   lower - A pointer the publicly visible representation of the "lower-half"
  *           driver state structure.
  *
  * Returned Values:
  *   Zero on success; a negated errno value on failure.
  *
  ****************************************************************************/

static int nrf52_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct nrf52_wdg_lowerhalf_s *priv = (FAR struct nrf52_wdg_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry: started=%d\n");
  DEBUGASSERT(priv);

  /* Have we already been started? */

  if (!priv->started)
    {
      flags = enter_critical_section();
      priv->lastreset = clock_systimer();
      priv->started   = true;
      nrf_wdt_int_enable(NRF_WDT_INT_TIMEOUT_MASK);
      nrf_wdt_task_trigger(NRF_WDT_TASK_START);
      leave_critical_section(flags);
    }

  return OK;
}

/****************************************************************************
  * Name: nrf52_stop
  *
  * Description:
  *   Stop the watchdog timer
  *
  * Input Parameters:
  *   lower - A pointer the publicly visible representation of the "lower-half"
  *           driver state structure.
  *
  * Returned Values:
  *   Zero on success; a negated errno value on failure.
  *
  ****************************************************************************/

static int nrf52_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  /* There is no way to disable the WDG timer once it has been started */

  wdinfo("Entry\n");
  return -ENOSYS;
}

/****************************************************************************
  * Name: nrf52_keepalive
  *
  * Description:
  *   Reset the watchdog timer to the current timeout value, prevent any
  *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
  *   the watchdog timer or "petting the dog".
  *
  * Input Parameters:
  *   lower - A pointer the publicly visible representation of the "lower-half"
  *           driver state structure.
  *
  * Returned Values:
  *   Zero on success; a negated errno value on failure.
  *
  ****************************************************************************/

static int nrf52_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct nrf52_wdg_lowerhalf_s *priv = (FAR struct nrf52_wdg_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  /* Reload the WDG timer */

  flags = enter_critical_section();

  priv->lastreset = clock_systimer();
  nrf_wdt_reload_request_set(0);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
  * Name: nrf52_getstatus
  *
  * Description:
  *   Get the current watchdog timer status
  *
  * Input Parameters:
  *   lower  - A pointer the publicly visible representation of the "lower-half"
  *            driver state structure.
  *   status - The location to return the watchdog status information.
  *
  * Returned Values:
  *   Zero on success; a negated errno value on failure.
  *
  ****************************************************************************/

static int nrf52_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct nrf52_wdg_lowerhalf_s *priv = (FAR struct nrf52_wdg_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks   = clock_systimer() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
    {
      elapsed = priv->timeout;
    }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n", status->timeout);
  wdinfo("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
  * Name: nrf52_settimeout
  *
  * Description:
  *   Set a new timeout value (and reset the watchdog timer)
  *
  * Input Parameters:
  *   lower   - A pointer the publicly visible representation of the "lower-half"
  *             driver state structure.
  *   timeout - The new timeout value in milliseconds.
  *
  * Returned Values:
  *   Zero on success; a negated errno value on failure.
  *
  ****************************************************************************/

static int nrf52_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct nrf52_wdg_lowerhalf_s *priv = (FAR struct nrf52_wdg_lowerhalf_s *)lower;

  wdinfo("Entry: timeout=%d\n", timeout);
  DEBUGASSERT(priv);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WDG_MAXTIMEOUT)
    {
      wderr("ERROR: Cannot represent timeout=%d > %d\n",
            timeout, WDG_MAXTIMEOUT);
      return -ERANGE;
    }

  if (priv->started)
    {
      wdwarn("WARNING: Watchdog is already started\n");
      return -EBUSY;
    }

  nrf_wdt_behaviour_set(priv->behaviour);
  priv->timeout = timeout;

  nrf_wdt_reload_value_set(((uint64_t) timeout * 32768) / 1000);

  up_enable_irq(NRF52_IRQ_WDT);

  wdinfo(" behaviour=%d  priority=%d\n", WDT_CONFIG_BEHAVIOUR, WDT_CONFIG_IRQ_PRIORITY);

  return OK;
}

/****************************************************************************
  * Public Functions
  ****************************************************************************/

/****************************************************************************
  * Name: nrf52_iwdginitialize
  *
  * Description:
  *   Initialize the WDG watchdog time.  The watchdog timer is initialized and
  *   registers as 'devpath.  The initial state of the watchdog time is
  *   disabled.
  *
  * Input Parameters:
  *   devpath - The full path to the watchdog.  This should be of the form
  *     /dev/watchdog0
  *
  * Returned Values:
  *   None
  *
  ****************************************************************************/

void nrf52_wdg_initialize(FAR const char *devpath, int16_t behaviour_sleep, int16_t behaviour_halt)
{
  FAR struct nrf52_wdg_lowerhalf_s *priv = &g_wdgdev;

  wdinfo("Entry: devpath=%s, behaviour_sleep=%d, behaviour_halt=%d\n", devpath, behaviour_sleep, behaviour_halt);

  /* Initialize the driver state structure. */
  priv->ops     = &g_wdgops;
  priv->started = false;
  priv->behaviour = (behaviour_halt << WDT_CONFIG_HALT_Pos) | (behaviour_sleep << WDT_CONFIG_SLEEP_Pos);

#if 0
  /* request LSECLK firstly */
  nrf52_clock_init();
  /* Select the lower power external 32,768Hz (Low-Speed External, LSE) oscillator
   * as RTC Clock Source and enable the Clock */
  nrf52_clock_lsclk_start();
#endif

  /* Register the watchdog driver as /dev/watchdog0 */
  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);

}

#endif /* CONFIG_WATCHDOG && CONFIG_nrf52_IWDG */
