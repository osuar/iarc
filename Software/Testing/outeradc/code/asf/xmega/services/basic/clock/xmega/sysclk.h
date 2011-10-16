/**
 * \file
 *
 * \brief Chip-specific system clock management functions
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
#ifndef XMEGA_SYSCLK_H_INCLUDED
#define XMEGA_SYSCLK_H_INCLUDED

#include <board.h>
#include <compiler.h>
#include <parts.h>
#include <ccp.h>
#include <osc.h>
#include <pll.h>

// Include clock configuration for the project.
#include <conf_clock.h>

#ifdef __cplusplus
extern "C" {
#endif

// Use 2 MHz with no prescaling if config was empty.
#ifndef CONFIG_SYSCLK_SOURCE
# define CONFIG_SYSCLK_SOURCE    SYSCLK_SRC_RC2MHZ
#endif /* CONFIG_SYSCLK_SOURCE */

#ifndef CONFIG_SYSCLK_PSADIV
# define CONFIG_SYSCLK_PSADIV    SYSCLK_PSADIV_1
#endif /* CONFIG_SYSCLK_PSADIV */

#ifndef CONFIG_SYSCLK_PSBCDIV
# define CONFIG_SYSCLK_PSBCDIV   SYSCLK_PSBCDIV_1_1
#endif /* CONFIG_SYSCLK_PSBCDIV */

/**
 * \weakgroup sysclk_group
 *
 * \section sysclk_group_config Configuration Symbols
 *
 * The following configuration symbols may be used to specify the
 * initial system clock configuration. If any of the symbols are not
 * set, reasonable defaults will be provided.
 *   - \b CONFIG_SYSCLK_SOURCE: The initial system clock source.
 *   - \b CONFIG_SYSCLK_PSADIV: The initial Prescaler A setting.
 *   - \b CONFIG_SYSCLK_PSBCDIV: The initial Prescaler B setting.
 *   - \b CONFIG_USBCLK_SOURCE: The initial USB clock source.
 *
 * @{
 */

//! \name System Clock Sources
//@{
//! Internal 2 MHz RC oscillator
#define SYSCLK_SRC_RC2MHZ    CLK_SCLKSEL_RC2M_gc
//! Internal 32 MHz RC oscillator
#define SYSCLK_SRC_RC32MHZ   CLK_SCLKSEL_RC32M_gc
//! Internal 32 KHz RC oscillator
#define SYSCLK_SRC_RC32KHZ   CLK_SCLKSEL_RC32K_gc
//! External oscillator
#define SYSCLK_SRC_XOSC      CLK_SCLKSEL_XOSC_gc
//! Phase-Locked Loop
#define SYSCLK_SRC_PLL       CLK_SCLKSEL_PLL_gc
//@}

//! \name Prescaler A Setting (relative to CLKsys)
//@{
#define SYSCLK_PSADIV_1      CLK_PSADIV_1_gc      //!< Do not prescale
#define SYSCLK_PSADIV_2      CLK_PSADIV_2_gc      //!< Prescale CLKper4 by 2
#define SYSCLK_PSADIV_4      CLK_PSADIV_4_gc      //!< Prescale CLKper4 by 4
#define SYSCLK_PSADIV_8      CLK_PSADIV_8_gc      //!< Prescale CLKper4 by 8
#define SYSCLK_PSADIV_16     CLK_PSADIV_16_gc     //!< Prescale CLKper4 by 16
#define SYSCLK_PSADIV_32     CLK_PSADIV_32_gc     //!< Prescale CLKper4 by 32
#define SYSCLK_PSADIV_64     CLK_PSADIV_64_gc     //!< Prescale CLKper4 by 64
#define SYSCLK_PSADIV_128    CLK_PSADIV_128_gc    //!< Prescale CLKper4 by 128
#define SYSCLK_PSADIV_256    CLK_PSADIV_256_gc    //!< Prescale CLKper4 by 256
#define SYSCLK_PSADIV_512    CLK_PSADIV_512_gc    //!< Prescale CLKper4 by 512
//@}

//! \name Prescaler B and C Setting (relative to CLKper4)
//@{
//! Do not prescale
#define SYSCLK_PSBCDIV_1_1   CLK_PSBCDIV_1_1_gc
//! Prescale CLKper and CLKcpu by 2
#define SYSCLK_PSBCDIV_1_2   CLK_PSBCDIV_1_2_gc
//! Prescale CLKper2, CLKper and CLKcpu by 4
#define SYSCLK_PSBCDIV_4_1   CLK_PSBCDIV_4_1_gc
//! Prescale CLKper2 by 2, CLKper and CLKcpu by 4
#define SYSCLK_PSBCDIV_2_2   CLK_PSBCDIV_2_2_gc
//@}

//! \name System Clock Port Numbers
enum sysclk_port_id {
	SYSCLK_PORT_GEN,   //!< Devices not associated with a specific port.
	SYSCLK_PORT_A,     //!< Devices on PORTA
	SYSCLK_PORT_B,     //!< Devices on PORTB
	SYSCLK_PORT_C,     //!< Devices on PORTC
	SYSCLK_PORT_D,     //!< Devices on PORTD
	SYSCLK_PORT_E,     //!< Devices on PORTE
	SYSCLK_PORT_F,     //!< Devices on PORTF
};

/*! \name Clocks not associated with any port
 *
 * \note See the datasheet for available modules in the device.
 */
//@{
#define SYSCLK_DMA        PR_DMA_bm     //!< DMA Controller
#define SYSCLK_EVSYS      PR_EVSYS_bm   //!< Event System
#define SYSCLK_RTC        PR_RTC_bm     //!< Real-Time Counter
#define SYSCLK_EBI        PR_EBI_bm     //!< Ext Bus Interface
#define SYSCLK_AES        PR_AES_bm     //!< AES Module
#define SYSCLK_USB        PR_USB_bm     //!< USB Module
//@}

/*! \name Clocks on PORTA and PORTB
 *
 * \note See the datasheet for available modules in the device.
 */
//@{
#define SYSCLK_AC         PR_AC_bm      //!< Analog Comparator
#define SYSCLK_ADC        PR_ADC_bm     //!< A/D Converter
#define SYSCLK_DAC        PR_DAC_bm     //!< D/A Converter
//@}

/*! \name Clocks on PORTC, PORTD, PORTE and PORTF
 *
 * \note See the datasheet for available modules in the device.
 */
//@{
#define SYSCLK_TC0        PR_TC0_bm      //!< Timer/Counter 0
#define SYSCLK_TC1        PR_TC1_bm      //!< Timer/Counter 1
#define SYSCLK_HIRES      PR_HIRES_bm    //!< Hi-Res Extension
#define SYSCLK_SPI        PR_SPI_bm      //!< SPI controller
#define SYSCLK_USART0     PR_USART0_bm   //!< USART 0
#define SYSCLK_USART1     PR_USART1_bm   //!< USART 1
#define SYSCLK_TWI        PR_TWI_bm      //!< TWI controller
//@}


#if XMEGA_USB
//! \name USB Clock Sources
//@{
//! Internal 32 MHz RC oscillator
#define USBCLK_SRC_RCOSC    CLK_USBSRC_RC32M_gc
//! Phase-Locked Loop
#define USBCLK_SRC_PLL      CLK_USBSRC_PLL_gc
//@}

/**
 * \def CONFIG_USBCLK_SOURCE
 * \brief Configuration symbol for the USB clock source
 *
 * If the device features an USB module, and this is intended to be used, this
 * symbol must be defined with the clock source configuration.
 *
 * Define this as one of the \c USBCLK_SRC_xxx definitions. If the PLL is
 * selected, it must be configured to run at 48 MHz. If the 32 MHz RC oscillator
 * is selected, it must be tuned to 48 MHz by means of the DFLL.
 */
#ifdef __DOXYGEN__
# define CONFIG_USBCLK_SOURCE
#endif

#endif // XMEGA_USB

#ifndef __ASSEMBLY__

/**
 * \name Querying the system clock and its derived clocks
 */
//@{

/**
 * \brief Return the current rate in Hz of the main system clock
 *
 * \todo This function assumes that the main clock source never changes
 * once it's been set up, and that PLL0 always runs at the compile-time
 * configured default rate. While this is probably the most common
 * configuration, which we want to support as a special case for
 * performance reasons, we will at some point need to support more
 * dynamic setups as well.
 */
static inline uint32_t sysclk_get_main_hz(void)
{
	switch (CONFIG_SYSCLK_SOURCE) {
	case SYSCLK_SRC_RC2MHZ:
		return 2000000UL;

	case SYSCLK_SRC_RC32MHZ:
#ifdef CONFIG_OSC_RC32_CAL
		return CONFIG_OSC_RC32_CAL;
#else
		return 32000000UL;
#endif

	case SYSCLK_SRC_RC32KHZ:
		return 32768UL;

#ifdef BOARD_XOSC_HZ
	case SYSCLK_SRC_XOSC:
		return BOARD_XOSC_HZ;
#endif

#ifdef CONFIG_PLL0_SOURCE
	case SYSCLK_SRC_PLL:
		return pll_get_default_rate(0);
#endif

	default:
		//unhandled_case(CONFIG_SYSCLK_SOURCE);
		return 0;
	}
}

/**
 * \brief Return the current rate in Hz of clk_PER4.
 *
 * This clock can run up to four times faster than the CPU clock.
 */
static inline uint32_t sysclk_get_per4_hz(void)
{
	uint8_t shift = 0;

	if (CONFIG_SYSCLK_PSADIV & (1U << CLK_PSADIV_gp)) {
		shift = (CONFIG_SYSCLK_PSADIV >> (1 + CLK_PSADIV_gp)) + 1;
	}

	return sysclk_get_main_hz() >> shift;
}

/**
 * \brief Return the current rate in Hz of clk_PER2.
 *
 * This clock can run up to two times faster than the CPU clock.
 */
static inline uint32_t sysclk_get_per2_hz(void)
{
	switch (CONFIG_SYSCLK_PSBCDIV) {
	case SYSCLK_PSBCDIV_1_1: /* Fall through */
	case SYSCLK_PSBCDIV_1_2:
		return sysclk_get_per4_hz();

	case SYSCLK_PSBCDIV_4_1:
		return sysclk_get_per4_hz() / 4;

	case SYSCLK_PSBCDIV_2_2:
		return sysclk_get_per4_hz() / 2;

	default:
		//unhandled_case(CONFIG_SYSCLK_PSBCDIV);
		return 0;
	}
}

/**
 * \brief Return the current rate in Hz of clk_PER.
 *
 * This clock always runs at the same rate as the CPU clock.
 */
static inline uint32_t sysclk_get_per_hz(void)
{
	if (CONFIG_SYSCLK_PSBCDIV & (1U << CLK_PSBCDIV_gp))
		return sysclk_get_per2_hz() / 2;
	else
		return sysclk_get_per2_hz();
}

/**
 * \brief Return the current rate in Hz of the CPU clock.
 */
static inline uint32_t sysclk_get_cpu_hz(void)
{
	return sysclk_get_per_hz();
}

//@}

//! \name Enabling and disabling synchronous clocks
//@{

/**
 * \brief Enable the clock to peripheral \a id on port \a port
 *
 * \param port ID of the port to which the module is connected (one of
 * the \c SYSCLK_PORT_* definitions).
 * \param id The ID (bitmask) of the peripheral module to be enabled.
 */
extern void sysclk_enable_module(enum sysclk_port_id port, uint8_t id);

/**
 * \brief Disable the clock to peripheral \a id on port \a port
 *
 * \param port ID of the port to which the module is connected (one of
 * the \c SYSCLK_PORT_* definitions).
 * \param id The ID (bitmask) of the peripheral module to be disabled.
 */
extern void sysclk_disable_module(enum sysclk_port_id port, uint8_t id);

/**
 * \brief Check if the synchronous clock is enabled for a module
 *
 * \param port ID of the port to which the module is connected (one of
 * the \c SYSCLK_PORT_* definitions).
 * \param id The ID (bitmask) of the peripheral module to check (one of
 * the \c SYSCLK_* module definitions).
 *
 * \retval true If the clock for module \a id on \a port is enabled.
 * \retval false If the clock for module \a id on \a port is disabled.
 */
static inline bool sysclk_module_is_enabled(enum sysclk_port_id port,
		uint8_t id)
{
	uint8_t mask = *((uint8_t *)&PR.PRGEN + port);
	return (mask & id) == 0;
}

#if defined(CONFIG_USBCLK_SOURCE) || defined(__DOXYGEN__)
extern void sysclk_enable_usb(uint8_t freq);
extern void sysclk_disable_usb(void);
#endif

//@}

//! \name System Clock Source and Prescaler configuration
//@{

/**
 * \brief Set system clock prescaler configuration
 *
 * This function will change the system clock prescaler configuration to
 * match the parameters.
 *
 * \note The parameters to this function are device-specific.
 *
 * \param psadiv The prescaler A setting (one of the \c SYSCLK_PSADIV_*
 * definitions). This determines the clkPER4 frequency.
 * \param psbcdiv The prescaler B and C settings (one of the \c SYSCLK_PSBCDIV_*
 * definitions). These determine the clkPER2, clkPER and clkCPU frequencies.
 */
static inline void sysclk_set_prescalers(uint8_t psadiv, uint8_t psbcdiv)
{
	ccp_write_io((uint8_t *)&CLK.PSCTRL, psadiv | psbcdiv);
}

/**
 * \brief Change the source of the main system clock.
 *
 * \param src The new system clock source. Must be one of the constants
 * from the <em>System Clock Sources</em> section.
 */
static inline void sysclk_set_source(uint8_t src)
{
	ccp_write_io((uint8_t *)&CLK.CTRL, src);
}

/**
 * \brief Lock the system clock configuration
 *
 * This function will lock the current system clock source and prescaler
 * configuration, preventing any further changes.
 */
static inline void sysclk_lock(void)
{
	ccp_write_io((uint8_t *)&CLK.LOCK, CLK_LOCK_bm);
}

//@}

//! \name System Clock Initialization
//@{

extern void sysclk_init(void);

//@}

#endif /* !__ASSEMBLY__ */

//! @}

#ifdef __cplusplus
}
#endif

#endif /* XMEGA_SYSCLK_H_INCLUDED */
