/**
 * \file
 *
 * \brief Chip-specific system clock management functions
 *
 * Copyright (C) 2009 Atmel Corporation. All rights reserved.
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
#include <compiler.h>
#include <stdbool.h>
#include <sysclk.h>
#include <avr32/io.h>
#include <flashc.h>
#include <gpio.h>


/**
 * \weakgroup sysclk_group
 * @{
 */

/**
 * \internal
 * \defgroup sysclk_internals_group System Clock internals
 *
 * System clock management is fairly straightforward apart from one
 * thing: Enabling and disabling bus bridges. When all peripherals on a
 * given bus are disabled, the bridge to the bus may be disabled. Only
 * the PBA and PBB busses support this, and it is not practical to
 * disable the PBA bridge as it includes the Power Manager, so turning
 * it off would make it impossible to turn anything back on again.
 *
 * The system clock implementation keeps track of a reference count for
 * PBB. When the reference count is zero, the bus bridge is disabled, otherwise
 * it is enabled.
 *
 * @{
 */
/**
 * \internal
 * \brief Enable a maskable module clock.
 * \param bus_id Bus index starting at 0 and following the same order as
 * the xxxMASK registers.
 * \param module_index Index of the module to be enabled. This is the
 * bit number in the corresponding xxxMASK register.
 */
void sysclk_priv_enable_module(unsigned int bus_id, unsigned int module_index)
{
  // Ahem... no, thanks.
}

/**
 * \internal
 * \brief Disable a maskable module clock.
 * \param bus_id Bus index starting at 0 and following the same order as
 * the xxxMASK registers.
 * \param module_index Index of the module to be disabled. This is the
 * bit number in the corresponding xxxMASK register.
 */
void sysclk_priv_disable_module(unsigned int bus_id, unsigned int module_index)
{
}


/**
 * \brief Disable a module clock derived from the PBB clock
 * \param index Index of the module clock in the PBBMASK register
 */
void sysclk_disable_pbb_module(unsigned int index)
{
}

#ifdef CONFIG_USBCLK_SOURCE
void sysclk_enable_usb(void)
{
}

void sysclk_disable_usb(void)
{
}
#endif // CONFIG_USBCLK_SOURCE


void sysclk_init(void)
{
  // Enable clock from fractional prescaler
  AVR32_PM.fpcvr = 0x80010001; // enable prescaler + div=min=2
  //AVR32_PM.fpcvr = 0x80000000 | (96<<16) | 125; // enable prescaler + Fout = 256*48000 KHz
  //AVR32_PM.fpcvr = 0x80000000 | (16<<16) | 125; // enable prescaler + Fout = 256*8000 KHz

  // Map Franctional prescaler output on generic clock (pin AH3 on Mistral)
  gpio_enable_module_pin( AVR32_PM_GCLK_3_0_PIN, AVR32_PM_GCLK_3_0_FUNCTION);

  // Map PLL output on generic clock CLK_GEN(0) (pin AL3 on Mistral)
  gpio_enable_module_pin( AVR32_PM_GCLK_0_0_PIN, AVR32_PM_GCLK_0_0_FUNCTION);
}

//! @}
