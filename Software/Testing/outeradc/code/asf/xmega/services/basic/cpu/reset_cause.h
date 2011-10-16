/**
 * \file
 *
 * \brief CPU reset cause functions
 *
 * Copyright (C) 2010 Atmel Corporation. All rights reserved.
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */
#ifndef COMMON_DRIVERS_CPU_RESET_CAUSE_H
#define COMMON_DRIVERS_CPU_RESET_CAUSE_H

#include <parts.h>
#include <stdbool.h>

#if defined(XMEGA)
# include "xmega_reset_cause.h"
#elif defined(__AVR32__) || defined(__ICCAVR32__)
# include "avr32_reset_cause.h"
#else
# error Unsupported chip type
#endif

/**
 * \defgroup reset_cause_group CPU reset cause
 *
 * This is a generic interface for getting and clearing the chip reset causes.
 *
 * \section dependencies Dependencies
 *
 * The reset cause interface does not depend on any other modules, as it only
 * accesses a few registers in the device core.
 *
 * On the other hand, the software reset call might depend on \ref sysclk_group
 * to enable the clock to the debug system, for devices doing software reset
 * through the on-chip debug system. This applies only to the 32-bit AVR
 * devices.
 *
 * @{
 */

/*
 * Sanity check of reset causes, define undefined reset causes to 0. Hence they
 * will always return false when querried.
 */
#ifndef CHIP_RESET_CAUSE_BOD_CPU
/**
 * \brief Brown-out detected on CPU power domain reset cause not available on
 * this chip.
 */
# define CHIP_RESET_CAUSE_BOD_CPU       0
#endif
#ifndef CHIP_RESET_CAUSE_BOD_IO
/**
 * \brief Brown-out detected on I/O power domain reset cause not available on
 * this chip.
 */
# define CHIP_RESET_CAUSE_BOD_IO        0
#endif
#ifndef CHIP_RESET_CAUSE_CPU_ERROR
//! CPU error reset cause not available on this chip.
# define CHIP_RESET_CAUSE_CPU_ERROR     0
#endif
#ifndef CHIP_RESET_CAUSE_EXTRST
//! External reset cause not available on this chip.
# define CHIP_RESET_CAUSE_EXTRST        0
#endif
#ifndef CHIP_RESET_CAUSE_JTAG
//! JTAG reset cause not available on this chip.
# define CHIP_RESET_CAUSE_JTAG          0
#endif
#ifndef CHIP_RESET_CAUSE_OCD
//! On-chip debug system reset cause not available on this chip.
# define CHIP_RESET_CAUSE_OCD           0
#endif
#ifndef CHIP_RESET_CAUSE_POR
//! Power-on-reset reset cause not available on this chip.
# define CHIP_RESET_CAUSE_POR           0
#endif
#ifndef CHIP_RESET_CAUSE_SLEEP
//! Wake from Shutdown sleep mode reset cause not available on this chip.
# define CHIP_RESET_CAUSE_SLEEP         0
#endif
#ifndef CHIP_RESET_CAUSE_SOFT
//! Software reset reset cause not available on this chip.
# define CHIP_RESET_CAUSE_SOFT          0
#endif
#ifndef CHIP_RESET_CAUSE_SPIKE
//! Spike detected reset cause not available on this chip.
# define CHIP_RESET_CAUSE_SPIKE         0
#endif
#ifndef CHIP_RESET_CAUSE_WDT
//! Watchdog timeout reset cause not available on this chip.
# define CHIP_RESET_CAUSE_WDT           0
#endif

/**
 * \brief List of reset causes in bit-mask format
 */
enum reset_cause {
	/** \brief Brown-out detected on CPU power domain reset cause */
	RESET_CAUSE_BOD_CPU     = CHIP_RESET_CAUSE_BOD_CPU,
	/** \brief Brown-out detected on I/O power domain reset cause */
	RESET_CAUSE_BOD_IO      = CHIP_RESET_CAUSE_BOD_IO,
	/** \brief CPU error reset cause */
	RESET_CAUSE_CPU_ERROR   = CHIP_RESET_CAUSE_CPU_ERROR,
	/** \brief External reset cause */
	RESET_CAUSE_EXTRST      = CHIP_RESET_CAUSE_EXTRST,
	/** \brief JTAG reset cause */
	RESET_CAUSE_JTAG        = CHIP_RESET_CAUSE_JTAG,
	/** \brief On-chip debug system reset cause */
	RESET_CAUSE_OCD         = CHIP_RESET_CAUSE_OCD,
	/** \brief Power-on-reset reset cause */
	RESET_CAUSE_POR         = CHIP_RESET_CAUSE_POR,
	/** \brief Wake from Shutdown sleep mode reset cause */
	RESET_CAUSE_SLEEP       = CHIP_RESET_CAUSE_SLEEP,
	/** \brief Software reset reset cause */
	RESET_CAUSE_SOFT        = CHIP_RESET_CAUSE_SOFT,
	/** \brief Spike detected reset cause */
	RESET_CAUSE_SPIKE       = CHIP_RESET_CAUSE_SPIKE,
	/** \brief Watchdog timeout reset cause */
	RESET_CAUSE_WDT         = CHIP_RESET_CAUSE_WDT,
};

//! \name Management
//@{

/**
 * \fn void reset_do_soft_reset(void)
 * \brief Perform a software reset of the device
 *
 * \note This function will never return.
 * \note This function does not disable interrupts, this is up to the caller to
 *       handle.
 */
/**
 * \fn reset_cause_t reset_cause_get_causes(void)
 * \brief Get all reset causes
 *
 * This function will return a value containing the currently triggered reset
 * cause(s).
 *
 * \return Bit-mask with each active reset cause set to 1.
 */
/**
 * \fn reset_cause_clear_causes(reset_cause_t causes)
 * \brief Clear a bit-mask of reset causes
 *
 * This function will clear the provided reset causes in the reset cause
 * register.
 *
 * \param causes bit-mask of reset causes to clear
 */

//@}

//! \name Specific reset cause helper functions
//@{

/**
 * \brief Check if chip reset was caused by a CPU power brown-out detection
 *
 * \return True if reset was caused by a CPU power brown-out detection
 */
static inline bool reset_cause_is_cpu_brown_out_detected(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_BOD_CPU);
}

/**
 * \brief Check if chip reset was caused by an I/O power brown-out detection
 *
 * \return True if reset was caused by an I/O power brown-out detection
 */
static inline bool reset_cause_is_io_brown_out_detected(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_BOD_IO);
}

/**
 * \brief Check if chip reset was caused by a brown-out detection on any
 * power domain.
 *
 * \return True if reset was caused by a power brown-out detection
 */
static inline bool reset_cause_is_brown_out_detected(void)
{
	return (reset_cause_is_cpu_brown_out_detected() ||
			reset_cause_is_io_brown_out_detected());
}

/**
 * \brief Check if chip reset was caused by a CPU error, illegal access
 *
 * \return True if reset was caused by a CPU error, illegal access
 */
static inline bool reset_cause_is_cpu_error(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_CPU_ERROR);
}

/**
 * \brief Check if chip reset was caused by an external reset
 *
 * \return True if reset was caused by an external reset
 */
static inline bool reset_cause_is_external_reset(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_EXTRST);
}

/**
 * \brief Check if chip reset was caused by a JTAG reset
 *
 * \return True if reset was caused by a JTAG reset
 */
static inline bool reset_cause_is_jtag(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_JTAG);
}

/**
 * \brief Check if chip reset was caused by the on-chip debug system
 *
 * \return True if reset was caused by the on-chip debug system
 */
static inline bool reset_cause_is_ocd(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_OCD);
}

/**
 * \brief Check if chip reset was caused by a power-on-reset
 *
 * \return True if reset was caused by a power-on-reset
 */
static inline bool reset_cause_is_power_on_reset(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_POR);
}

/**
 * \brief Check if chip reset was caused by a wake up from shutdown sleep mode
 *
 * \return True if reset was caused by a wake up from shutdown sleep mode
 */
static inline bool reset_cause_is_wake_from_shutdown_sleep(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_SLEEP);
}

/**
 * \brief Check if chip reset was caused by a software reset
 *
 * \return True if reset was caused by a software reset
 */
static inline bool reset_cause_is_software_reset(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_SOFT);
}

/**
 * \brief Check if chip reset was caused by a power spike detection
 *
 * \return True if reset was caused by a spike detection
 */
static inline bool reset_cause_is_spike_detected(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_SPIKE);
}

/**
 * \brief Check if chip reset was caused by a watchdog timeout
 *
 * \return True if reset was caused by a watchdog timeout
 */
static inline bool reset_cause_is_watchdog(void)
{
	return (reset_cause_get_causes() & RESET_CAUSE_WDT);
}

//@}

//! @}

#endif /* COMMON_DRIVERS_CPU_RESET_CAUSE_H */
