/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief This file controls the software FIFO management.
 *
 * These functions manages FIFOs thanks to simple a API. The FIFO can
 * be 100% full thanks to a double-index range implementation. For example,
 * a FIFO of 4 elements can be implemented: the FIFO can really hold up to 4
 * elements.
 * This is particurly well suited for any kind of application needing queuing
 * data, events, ...
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ***************************************************************************/

/* Copyright (c) 2010 Atmel Corporation. All rights reserved.
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

#ifndef _FIFO_H_
#define _FIFO_H_

//! Error codes used by FIFO driver.
enum {
	FIFO_OK = 0, //!< Normal operation.
	FIFO_ERROR_OVERFLOW, //!< Attempt to push something in a FIFO that is full.
	FIFO_ERROR_UNDERFLOW, //!< Attempt to pull something from a FIFO that is empty
	FIFO_ERROR
//!< Error (malloc failed, ...)
};

//! Size of the element
enum {
	FIFO_ELEMENT_8BITS = 0, //!< Element is 8 bits
	FIFO_ELEMENT_16BITS, //!< Element is 16 bits
	FIFO_ELEMENT_32BITS
//!< Element is 32 bits
};

//! FIFO descriptor used by FIFO driver.
typedef struct {
	volatile UnionVPtr buffer;
	volatile uint16_t rd_id;
	volatile uint16_t wr_id;
	uint16_t size;
	uint8_t align;
} fifo_desc_t;

//! @brief This function initializes a new software FIFO for a certain 'size'.
//! Both fifo descriptor and buffer must be allocated by the caller before calling
//! this function.
//!
//! @param fifo_desc  Pointer on the FIFO descriptor.
//! @param buffer     Pointer on the buffer.
//! @param size       Size of the buffer (unit is in number of 'item').
//!                   It must be a 2-power.
//! @param fifo_desc  The size of the element.
//!   @arg FIFO_ELEMENT_8BITS
//!   @arg FIFO_ELEMENT_16BITS
//!   @arg FIFO_ELEMENT_32BITS
//!
//! @return Status
//!   @retval FIFO_OK when no error occured.
//!   @retval FIFO_ERROR when the size is not a 2-power.
//!
int fifo_init_no_malloc(fifo_desc_t *fifo_desc, void *buffer, uint16_t size,
      uint16_t element_size);

//! @brief This function initializes a new software FIFO for a certain 'size'.
//! Both fifo descriptor and buffer are allocated by the function.
//!
//! @param fifo_desc  The FIFO descriptor.
//! @param size       Size of the buffer (unit is in number of 'item').
//!                   It must be a 2-power.
//! @param fifo_desc  The size of the element.
//!   @arg FIFO_ELEMENT_8BITS
//!   @arg FIFO_ELEMENT_16BITS
//!   @arg FIFO_ELEMENT_32BITS
//!
//! @return Status
//!   @retval FIFO_OK when no error occured.
//!   @retval FIFO_ERROR when the buffer can not be allocated or if the size
//!           is not a 2-power.
//!
extern int fifo_init_malloc(fifo_desc_t **fifo_desc, uint16_t size,
      uint16_t element_size);

//! @brief This function stops a software FIFO and free the allocated buffer.
//!
//! @param fifo_desc  The FIFO descriptor.
//!
extern void fifo_stop_malloc(fifo_desc_t **fifo_desc);

//! @brief This function returns the number of elements in the FIFO.
//!
//! @param fifo_desc  The FIFO descriptor.
//!
//! @return The number of used elements.
//!
extern uint16_t fifo_get_used_size(fifo_desc_t *fifo_desc);

//! @brief This function returns the remaining free spaces of the FIFO (in number of elements).
//!
//! @param fifo_desc  The FIFO descriptor.
//!
//! @return The number of free elements.
//!
extern uint16_t fifo_get_free_size(fifo_desc_t *fifo_desc);

//! @brief This function pushes a new element in the FIFO.
//!
//! @param fifo_desc  The FIFO descriptor.
//! @param item       element to push.
//!
//! @return Status
//!   @retval FIFO_OK when no error occured.
//!   @retval FIFO_ERROR_OVERFLOW when the FIFO was already full before pushing the element.
//!
extern int fifo_push(fifo_desc_t *fifo_desc, uint32_t item);

//! @brief This function gets a new element from the FIFO.
//!
//! @param fifo_desc  The FIFO descriptor.
//! @param item       extracted element.
//!
//! @return Status
//!   @retval FIFO_OK when no error occured.
//!   @retval FIFO_ERROR_UNDERFLOW when the FIFO was empty.
//!
extern int fifo_pull(fifo_desc_t *fifo_desc, void *item);

//! @brief This function resets a software FIFO.
//!
//! @param fifo_desc  The FIFO descriptor.
extern void fifo_reset(fifo_desc_t *fifo_desc);
#endif  // _FIFO_H_
