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
#ifndef CHIP_SYSCLK_H_INCLUDED
#define CHIP_SYSCLK_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define FCPU_HZ_MISTRAL 20000000
#define FCPU_HZ_ZEPHYR  48000000

#define FCPU_HZ            FCPU_HZ_ZEPHYR
#define FPBA_HZ            FCPU_HZ

/**
 * \weakgroup sysclk_group
 * @{
 */

//! \name System Clock Sources
//@{
#define SYSCLK_SRC_RCSYS         0    //!< Use slow clock as main clock
#define SYSCLK_SRC_OSC0          1    //!< Use OSC0 as main clock
#define SYSCLK_SRC_PLL0          2    //!< Use PLL0 as main clock
#define SYSCLK_SRC_ZEPHYR_CLOCK  3    //!< Use Zephyr hardcoded clock
//@}

//! \name USB Clock Sources
//@{
#define USBCLK_SRC_OSC0     0    //!< Use OSC0
#define USBCLK_SRC_PLL0     1    //!< Use PLL0
#define USBCLK_SRC_PLL1     2    //!< Use PLL1
//@}


//! \name Clocks derived from the CPU clock
//@{
#define SYSCLK_OCD        AVR32_OCD_CLK_CPU         //!< On-Chip Debug system
#define SYSCLK_SYSTIMER   AVR32_CORE_CLK_CPU_COUNT  //!< COUNT/COMPARE registers
//@}

//! \name Clocks derived from the HSB clock
//@{
//! Flash data interface
#define SYSCLK_FLASHC_DATA  (AVR32_FLASHC_CLK_HSB % 32)
//! HSB<->PBA bridge
#define SYSCLK_PBA_BRIDGE   (AVR32_HMATRIX_CLK_HSB_PBA_BRIDGE % 32)
//! HSB<->PBB bridge
#define SYSCLK_PBB_BRIDGE   (AVR32_HMATRIX_CLK_HSB_PBB_BRIDGE % 32)
//! USB DMA and FIFO interface
#define SYSCLK_USBC_DATA    (AVR32_USBC_CLK_HSB % 32)
//! MACB DMA interface
#define SYSCLK_MACB_DATA    (AVR32_MACB_CLK_HSB % 32)
//! PDCA memory interface
#define SYSCLK_PDCA_HSB     (AVR32_PDCA_CLK_HSB % 32)
//! External Bus Interface
#define SYSCLK_EBI          (AVR32_EBI_CLK_HSB % 32)
//@}

//! \name Clocks derived from the PBA clock
//@{
//! Internal interrupt controller
#define SYSCLK_INTC         (AVR32_INTC_CLK_PBA % 32)
//! General-Purpose I/O
#define SYSCLK_GPIO         (AVR32_GPIO_CLK_PBA % 32)
//! PDCA periph bus interface
#define SYSCLK_PDCA_PB      (AVR32_PDCA_CLK_PBA % 32)
//! PM/RTC/EIM configuration
#define SYSCLK_PM           (AVR32_PM_CLK_PBA % 32)
//! A/D Converter
#define SYSCLK_ADC          (AVR32_ADC_CLK_PBA % 32)
//! SPI Controller 0
#define SYSCLK_SPI0         (AVR32_SPI0_CLK_PBA % 32)
//! SPI Controller 1
#define SYSCLK_SPI1         (AVR32_SPI1_CLK_PBA % 32)
//! TWI Controller
#define SYSCLK_TWI          (AVR32_TWI_CLK_PBA % 32)
//! USART 0
#define SYSCLK_USART0       (AVR32_USART0_CLK_PBA % 32)
//! USART 1
#define SYSCLK_USART1       (AVR32_USART1_CLK_PBA % 32)
//! USART 2
#define SYSCLK_USART2       (AVR32_USART2_CLK_PBA % 32)
//! USART 3
#define SYSCLK_USART3       (AVR32_USART3_CLK_PBA % 32)
//! PWM
#define SYSCLK_PWM          (AVR32_PWM_CLK_PBA % 32)
//! Synchronous Serial Controller
#define SYSCLK_SSC          (AVR32_SSC_CLK_PBA % 32)
//! Timer/Counter
#define SYSCLK_TC           (AVR32_TC_CLK_PBA % 32)
//! D/A Converter
#define SYSCLK_DAC          (AVR32_ABDAC_CLK_PBA % 32)
//@}

//! \name Clocks derived from the PBB clock
//@{
//! HSB Matrix configuration
#define SYSCLK_HMATRIX      (AVR32_HMATRIX_CLK_PBB % 32)
//! USBC registers
#define SYSCLK_USBC_REGS    (AVR32_USBC_CLK_PBB % 32)
//! Flash Controller registers
#define SYSCLK_FLASHC_REGS  (AVR32_FLASHC_CLK_PBB % 32)
//! MACB Controller registers
#define SYSCLK_MACB_REGS    (AVR32_MACB_CLK_PBB % 32)
//! Static Memory Controller registers
#define SYSCLK_SMC_REGS     (AVR32_SMC_CLK_PBB % 32)
//! SDRAM Controller registers
#define SYSCLK_SDRAMC_REGS  (AVR32_SDRAMC_CLK_PBB % 32)
//@}

#ifndef __ASSEMBLY__

#include <avr32/io.h>

// Use the slow clock (RCOSC) with no prescaling if config was empty.
#ifndef CONFIG_SYSCLK_SOURCE
# define CONFIG_SYSCLK_SOURCE    SYSCLK_SRC_RCSYS
#endif /* CONFIG_SYSCLK_SOURCE */

/**
 * \def CONFIG_SYSCLK_CPU_DIV
 * \brief Configuration symbol for dividing the CPU clock frequency by
 * \f$2^{CONFIG\_SYSCLK\_CPU\_DIV}\f$
 *
 * If this symbol is not defined, the CPU clock frequency is not divided.
 *
 * This symbol may be defined in \ref conf_clock.h.
 */
#ifndef CONFIG_SYSCLK_CPU_DIV
# define CONFIG_SYSCLK_CPU_DIV   0
#endif /* CONFIG_SYSCLK_CPU_DIV */

/**
 * \def CONFIG_SYSCLK_PBA_DIV
 * \brief Configuration symbol for dividing the PBA clock frequency by
 * \f$2^{CONFIG\_SYSCLK\_PBA\_DIV}\f$
 *
 * If this symbol is not defined, the PBA clock frequency is not divided.
 *
 * This symbol may be defined in \ref conf_clock.h.
 */
#ifndef CONFIG_SYSCLK_PBA_DIV
# define CONFIG_SYSCLK_PBA_DIV   0
#endif /* CONFIG_SYSCLK_PBA_DIV */

/**
 * \name Querying the system clock and its derived clocks
 *
 * The following functions may be used to query the current frequency of
 * the system clock and the CPU and bus clocks derived from it.
 * sysclk_get_main_hz() and sysclk_get_cpu_hz() can be assumed to be
 * available on all platforms, although some platforms may define
 * additional accessors for various chip-internal bus clocks. These are
 * usually not intended to be queried directly by generic code.
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
	case SYSCLK_SRC_ZEPHYR_CLOCK:
    return FCPU_HZ_ZEPHYR;

#ifdef BOARD_OSC0_HZ
	case SYSCLK_SRC_OSC0:
		return BOARD_OSC0_HZ;
#endif

#ifdef CONFIG_PLL0_SOURCE
	case SYSCLK_SRC_PLL0:
		return pll_get_default_rate(0);
#endif

	default:
		/* unhandled_case(CONFIG_SYSCLK_SOURCE); */
		return 0;
	}
}

/**
 * \brief Return the current rate in Hz of the CPU clock
 *
 * \todo This function assumes that the CPU always runs at the system
 * clock frequency. We want to support at least two more scenarios:
 * Fixed CPU/bus clock dividers (config symbols) and dynamic CPU/bus
 * clock dividers (which may change at run time). Ditto for all the bus
 * clocks.
 */
static inline uint32_t sysclk_get_cpu_hz(void)
{
	return sysclk_get_main_hz() >> CONFIG_SYSCLK_CPU_DIV;
}

/**
 * \brief Return the current rate in Hz of the High-Speed Bus clock
 */
static inline uint32_t sysclk_get_hsb_hz(void)
{
	return sysclk_get_main_hz() >> CONFIG_SYSCLK_CPU_DIV;
}

/**
 * \brief Return the current rate in Hz of the Peripheral Bus A clock
 */
static inline uint32_t sysclk_get_pba_hz(void)
{
	return sysclk_get_main_hz() >> CONFIG_SYSCLK_PBA_DIV;
}

/**
 * \brief Return the current rate in Hz of the Peripheral Bus B clock
 */
static inline uint32_t sysclk_get_pbb_hz(void)
{
	return sysclk_get_main_hz() >> CONFIG_SYSCLK_CPU_DIV;
}

//@}

extern void sysclk_priv_enable_module(unsigned int bus_id,
		unsigned int module_index);
extern void sysclk_priv_disable_module(unsigned int bus_id,
		unsigned int module_index);

//! \name Enabling and disabling synchronous clocks
//@{

/**
 * \brief Enable a module clock derived from the CPU clock
 * \param index Index of the module clock in the CPUMASK register
 */
static inline void sysclk_enable_cpu_module(unsigned int index)
{
	sysclk_priv_enable_module(AVR32_PM_CLK_GRP_CPU, index);
}

/**
 * \brief Disable a module clock derived from the CPU clock
 * \param index Index of the module clock in the CPUMASK register
 */
static inline void sysclk_disable_cpu_module(unsigned int index)
{
	sysclk_priv_disable_module(AVR32_PM_CLK_GRP_CPU, index);
}

/**
 * \brief Enable a module clock derived from the HSB clock
 * \param index Index of the module clock in the HSBMASK register
 */
static inline void sysclk_enable_hsb_module(unsigned int index)
{
	sysclk_priv_enable_module(AVR32_PM_CLK_GRP_HSB, index);
}

/**
 * \brief Disable a module clock derived from the HSB clock
 * \param index Index of the module clock in the HSBMASK register
 */
static inline void sysclk_disable_hsb_module(unsigned int index)
{
	sysclk_priv_disable_module(AVR32_PM_CLK_GRP_HSB, index);
}

/**
 * \brief Enable a module clock derived from the PBA clock
 * \param index Index of the module clock in the PBAMASK register
 */
static inline void sysclk_enable_pba_module(unsigned int index)
{
	sysclk_priv_enable_module(AVR32_PM_CLK_GRP_PBA, index);
}

/**
 * \brief Disable a module clock derived from the PBA clock
 * \param index Index of the module clock in the PBAMASK register
 */
static inline void sysclk_disable_pba_module(unsigned int index)
{
	sysclk_priv_disable_module(AVR32_PM_CLK_GRP_PBA, index);
}

extern void sysclk_enable_pbb_module(unsigned int index);
extern void sysclk_disable_pbb_module(unsigned int index);

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
 * \param cpu_shift The CPU clock will be divided by \f$2^{cpu\_shift}\f$
 * \param pba_shift The PBA clock will be divided by \f$2^{pba\_shift}\f$
 * \param pbb_shift The PBB clock will be divided by \f$2^{pbb\_shift}\f$
 */
static inline void sysclk_set_prescalers(unsigned int cpu_shift,
		unsigned int pba_shift, unsigned int pbb_shift)
{
	uint32_t cksel = 0;

	Assert(cpu_shift <= pba_shift);
	Assert(cpu_shift <= pbb_shift);

	if (cpu_shift > 0)
		cksel = ((cpu_shift - 1) << AVR32_PM_CKSEL_CPUSEL)
				| (1U << AVR32_PM_CKSEL_CPUDIV);

	if (pba_shift > 0)
		cksel |= ((pba_shift - 1) << AVR32_PM_CKSEL_PBASEL)
				| (1U << AVR32_PM_CKSEL_PBADIV);

	if (pbb_shift > 0)
		cksel |= ((pbb_shift - 1) << AVR32_PM_CKSEL_PBBSEL)
				| (1U << AVR32_PM_CKSEL_PBBDIV);

	AVR32_PM.cksel = cksel;
}

/**
 * \brief Change the source of the main system clock.
 *
 * \pre The appropriate Flash Wait state must be set previously.
 *
 * \param src The new system clock source. Must be one of the constants
 * from the <em>System Clock Sources</em> section.
 */
static inline void sysclk_set_source(uint_fast8_t src)
{
	irqflags_t flags;
	uint32_t   mcctrl;

	Assert(src <= SYSCLK_SRC_PLL0);

	flags = cpu_irq_save();
	mcctrl = AVR32_PM.mcctrl & ~AVR32_PM_MCCTRL_MCSEL_MASK;
	mcctrl |= src << AVR32_PM_MCCTRL_MCSEL;
	AVR32_PM.mcctrl = mcctrl;
	cpu_irq_restore(flags);
}

//@}

/**
 * \brief Enables the USB generick clock
 * This one must be at 48MHz
 */
extern void sysclk_enable_usb(void);

/**
 * \brief Disables the USB generick clock
 */
extern void sysclk_disable_usb(void);

extern void sysclk_init(void);

#endif /* !__ASSEMBLY__ */

//! @}

#ifdef __cplusplus
}
#endif

#endif /* CHIP_SYSCLK_H_INCLUDED */
