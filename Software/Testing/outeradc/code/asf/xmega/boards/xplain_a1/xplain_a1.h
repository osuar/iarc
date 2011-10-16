/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief XPLAIN-A1 board header file.
 *
 * This file contains definitions and services related to the features of the
 * XPLAIN-A1 board.
 *
 * To use this board, define BOARD=XPLAIN_A1.
 *
 * - Compiler:           IAR EWAVR and GNU GCC for AVR
 * - Supported devices:  All ATxmegaA devices can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/**
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
#ifndef _XPLAIN_A1_H_
#define _XPLAIN_A1_H_

#include "compiler.h"

/*! \name Miscellaneous data
 */
//! @{
//! @}

//! Number of LEDs.
#define LED_COUNT   8

/*! \name GPIO Connections of LEDs
 */
//! @{
#  define LED0_GPIO   IOPORT_CREATE_PIN(PORTE,0)
#  define LED1_GPIO   IOPORT_CREATE_PIN(PORTE,1)
#  define LED2_GPIO   IOPORT_CREATE_PIN(PORTE,2)
#  define LED3_GPIO   IOPORT_CREATE_PIN(PORTE,3)
#  define LED4_GPIO   IOPORT_CREATE_PIN(PORTE,4)
#  define LED5_GPIO   IOPORT_CREATE_PIN(PORTE,5)
#  define LED6_GPIO   IOPORT_CREATE_PIN(PORTE,6)
#  define LED7_GPIO   IOPORT_CREATE_PIN(PORTE,7)
#  define LEDUSB_GPIO IOPORT_CREATE_PIN(PORTD,4)
//! @}


/*! \name GPIO Connections of Push Buttons
 */
//! @{
#  define GPIO_PUSH_BUTTON_0	IOPORT_CREATE_PIN(PORTF,0)
#  define GPIO_PUSH_BUTTON_1	IOPORT_CREATE_PIN(PORTF,1)
#  define GPIO_PUSH_BUTTON_2	IOPORT_CREATE_PIN(PORTF,2)
#  define GPIO_PUSH_BUTTON_3	IOPORT_CREATE_PIN(PORTF,3)
#  define GPIO_PUSH_BUTTON_4	IOPORT_CREATE_PIN(PORTF,4)
#  define GPIO_PUSH_BUTTON_5	IOPORT_CREATE_PIN(PORTF,5)
#  define GPIO_PUSH_BUTTON_6	IOPORT_CREATE_PIN(PORTF,6)
#  define GPIO_PUSH_BUTTON_7	IOPORT_CREATE_PIN(PORTF,7)
//! @}

/*! \name SPI Connections of the AT45DBX Data Flash Memory
 * TODO Change configuration 
 */
//! @{
#define AT45DBX_SPI           &USARTD0
#define AT45DBX_CS            IOPORT_CREATE_PIN(PORTD,0)
#define AT45DBX_MASTER_SCK    IOPORT_CREATE_PIN(PORTD,1)  // SCK as output
#define AT45DBX_MASTER_MOSI   IOPORT_CREATE_PIN(PORTD,3)  // MOSI as output
#define AT45DBX_SPI_CLK_MASK  SYSCLK_PORT_D

//! @}

/*! \name External oscillator
 */
//@{
#define BOARD_XOSC_HZ          8000000
#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL
#define BOARD_XOSC_STARTUP_US  100
//@}

#endif  // _XPLAIN_A1_H_
