/*
  flash.h - Flash memory methods
  Part of Grbl

  Copyright (c) 2021 by Michael Joya

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef flash_h
#define flash_h

unsigned char flash_get_char(unsigned int addr);


void flash_put_char(unsigned int addr, unsigned char new_value);


void memcpy_to_flash_with_checksum(unsigned int destination, char *source, unsigned int size);


int memcpy_from_flash_with_checksum(char *destination, unsigned int source, unsigned int size);

#endif
