/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Standard board header file.
 *
 * This file includes the appropriate board header file according to the
 * defined board (parameter BOARD).
 *
 * - Compiler:           IAR EWAVR/IAR EWAVR32 and GNU GCC for AVR or AVR32
 * - Supported devices:  All AVR devices can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "compiler.h"

/*! \name Base Boards
 */
//! @{
#define EVK1100           1   //!< AT32UC3A EVK1100 board.
#define EVK1101           2   //!< AT32UC3B EVK1101 board.
#define UC3C_EK           3   //!< AT32UC3C UC3C_EK board.
#define EVK1104           4   //!< AT32UC3A3 EVK1104 board.
#define EVK1105           5   //!< AT32UC3A EVK1105 board.
#define STK600_RCUC3L0    6   //!< STK600 RCUC3L0 board.
#define UC3L_EK           7   //!< AT32UC3L-EK board.
#define XPLAIN            8   //!< ATxmega128A1 Xplain board
#define XPLAIN_A1         9   //!< ATxmega128A1U Xplain-A1 board
#define STK600_RC064X     10  //!< ATxmega256A3 STK600 board
#define STK600_RC100X     11  //!< ATxmega128A1 STK600 board

#define USER_BOARD        99  //!< User-reserved board (if any).

//! @}

/*! \name Extension Boards
 */
//! @{
#define EXT1102           1   //!< AT32UC3B EXT1102 board.
#define MC300             2   //!< AT32UC3 MC300 board.
#define USER_EXT_BOARD    99  //!< User-reserved extension board (if any).
//! @}

#if BOARD == EVK1100
  #include "evk1100/evk1100.h"
#elif BOARD == EVK1101
  #include "evk1101/evk1101.h"
#elif BOARD == UC3C_EK
  #include "uc3c_ek/uc3c_ek.h"
#elif BOARD == EVK1104
  #include "evk1104/evk1104.h"
#elif BOARD == EVK1105
  #include "evk1105/evk1105.h"
#elif BOARD == STK600_RCUC3L0
  #include "stk600/rcuc3l0/stk600_rcuc3l0.h"
#elif BOARD == UC3L_EK
  #include "uc3l_ek/uc3l_ek.h"
#elif BOARD == XPLAIN
  #include "xplain/xplain.h"
#elif BOARD == XPLAIN_A1
  #include "xplain_a1/xplain_a1.h"
#elif BOARD == STK600_RC064X
  #include "stk600/rc064x/stk600_rc064x.h"
#elif BOARD == STK600_RC100X
  #include "stk600/rc100x/stk600_rc100x.h"
#elif BOARD == USER_BOARD
  // User-reserved area: #include the header file of your board here (if any).
  #include "user_board.h"
#else
  #error No known AVR board defined
#endif

#if (defined EXT_BOARD)
  #if EXT_BOARD == MC300
    #include "mc300/mc300.h"
  #elif EXT_BOARD == USER_EXT_BOARD
    // User-reserved area: #include the header file of your extension board here
    // (if any).
  #endif
#endif

#ifndef FRCOSC
  #define FRCOSC    AVR32_PM_RCOSC_FREQUENCY  //!< Default RCOsc frequency.
#endif

#if (defined(__GNUC__) && defined(__AVR32__)) || (defined(__ICCAVR32__) || defined(__AAVR32__))
#ifdef __AVR32_ABI_COMPILER__ // Automatically defined when compiling for AVR32, not when assembling.

/*! \brief This function initializes the board target ressources 
 *
 * This function should be called to ensure proper initialization of the target 
 * board hardware connected to the part.
 */
extern void board_init(void);

#endif  // #ifdef __AVR32_ABI_COMPILER__
#else
/*! \brief This function initializes the board target ressources 
 *
 * This function should be called to ensure proper initialization of the target 
 * board hardware connected to the part.
 */
extern void board_init(void);
#endif
#endif  // _BOARD_H_
