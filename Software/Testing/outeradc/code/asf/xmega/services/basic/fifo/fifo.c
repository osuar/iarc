/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief This file controls the software FIFO management.
 *
 * These functions allow to use FIFO thanks to simple APIs. The FIFO can
 * be 100% full thanks to a double-index range implementation.
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

#include "compiler.h"
#include "fifo.h"

int fifo_init_no_malloc(fifo_desc_t *fifo_desc, void *buffer, uint16_t size,
      uint16_t element_size)
{
	uint16_t tmp;

	// Check the size parameter. It must be a 2-power.
	tmp = size >> ctz(size);
	if (tmp != 1) {
		// We prefer catching this critical situation that way since most of users does not
		// even test return value...
		while (1) {
		}
		//return FIFO_ERROR;
	}

	// Keep the alignement
	fifo_desc->align = element_size;

	// Fifo starts empty.
	fifo_desc->rd_id = fifo_desc->wr_id = 0;

	// Save the size parameter.
	fifo_desc->size = size;

	// Save the buffer pointer
	fifo_desc->buffer.u8ptr = buffer;

	return FIFO_OK;
}

int fifo_init_malloc(fifo_desc_t **fifo_desc, uint16_t size,
      uint16_t element_size)
{
	uint16_t tmp;

	// Check the size parameter. It must be a 2-power.
	tmp = size >> ctz(size);
	if (tmp != 1) {
		// We prefer catching this critical situation that way since most of users does not
		// even test return value...
		while (1) {
		}
		//return FIFO_ERROR;
	}

	if (!(*fifo_desc = malloc(sizeof(fifo_desc_t))))
		return FIFO_ERROR;

	// Allocate memory for the buffer.
	if (!((*fifo_desc)->buffer.u8ptr = malloc(size))) {
		free(*fifo_desc);
		return FIFO_ERROR;
	}

	// Keep the alignement
	(*fifo_desc)->align = element_size;

	// Fifo starts empty.
	(*fifo_desc)->rd_id = (*fifo_desc)->wr_id = 0;

	// Save the size parameter.
	(*fifo_desc)->size = size;

	return FIFO_OK;
}

void fifo_reset(fifo_desc_t *fifo_desc)
{
	// Fifo starts empty.
	fifo_desc->rd_id = fifo_desc->wr_id = 0;
}

void fifo_stop_malloc(fifo_desc_t **fifo_desc)
{
	// Free allocated memory
	if ((*fifo_desc)->buffer.u8ptr) {
		free((void*) (*fifo_desc)->buffer.u8ptr);
		(*fifo_desc)->buffer.u8ptr = NULL;
	}
	if (*fifo_desc) {
		free((void*) *fifo_desc);
		*fifo_desc = NULL;
	}
}

uint16_t fifo_get_used_size(fifo_desc_t *fifo_desc)
{
	uint16_t val;
	val = fifo_desc->wr_id + 2 * fifo_desc->size;
	val -= fifo_desc->rd_id;
	return val & ((2 * fifo_desc->size) - 1);
}

uint16_t fifo_get_free_size(fifo_desc_t *fifo_desc)
{
	return fifo_desc->size - fifo_get_used_size(fifo_desc);
}

int fifo_push(fifo_desc_t *fifo_desc, uint32_t item)
{
	uint8_t wr_id;
	if (fifo_get_free_size(fifo_desc) == 0)
		return FIFO_ERROR_OVERFLOW;

	wr_id = fifo_desc->wr_id;

	if (fifo_desc->align == FIFO_ELEMENT_8BITS)
		fifo_desc->buffer.u8ptr[wr_id & (fifo_desc->size - 1)] = item;
	else if (fifo_desc->align == FIFO_ELEMENT_16BITS)
		fifo_desc->buffer.u16ptr[wr_id & (fifo_desc->size - 1)] = item;
	else
		// if( fifo_desc->align==FIFO_ELEMENT_32BITS )
		fifo_desc->buffer.u32ptr[wr_id & (fifo_desc->size - 1)] = item;

	// Must be the last thing to do.
	fifo_desc->wr_id = (wr_id + 1) & ((2 * fifo_desc->size) - 1);
	return FIFO_OK;
}

int fifo_pull(fifo_desc_t *fifo_desc, void *item)
{
	uint8_t rd_id;
	if (fifo_get_used_size(fifo_desc) == 0)
		return FIFO_ERROR_UNDERFLOW;

	rd_id = fifo_desc->rd_id;
	if (fifo_desc->align == FIFO_ELEMENT_8BITS)
		*(uint8_t*) item = fifo_desc->buffer.u8ptr[rd_id & (fifo_desc->size - 1)];
	else if (fifo_desc->align == FIFO_ELEMENT_16BITS)
		*(uint16_t*) item = fifo_desc->buffer.u16ptr[rd_id
		      & (fifo_desc->size - 1)];
	else
		// if( fifo_desc->align==FIFO_ELEMENT_32BITS )
		*(uint32_t*) item = fifo_desc->buffer.u32ptr[rd_id
		      & (fifo_desc->size - 1)];

	// Must be the last thing to do.
	fifo_desc->rd_id = (rd_id + 1) & ((2 * fifo_desc->size) - 1);
	return FIFO_OK;
}
