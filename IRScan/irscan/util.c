/** 
 * @file
 * @author Joey Tomlinson
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 */

#include <avr/io.h>
#include <stdlib.h>

/**
 * Copies a char to another char, using a mask.
 * 
 * @param source pointer to the source to copy from
 * @param dest pointer to the destination to copy to
 * @param mask mask to use while copying. 0 = masked 1 = not masked
 * bits of dest which have a 0 in the corresponding mask position are not affected
 * 
 */

void cmaskedCopy (char source, char * dest, char mask) {
	
	source &= mask; // Set masked bits in source = 0
	*dest |= source; // Whenever there is a 1 in source, set a 1 in dest
	
	source |= ~mask; // Set masked bits in source = 1
	*dest &= source; // Wherever there is a 0 in source, set 0 in TCCR1B 
}
