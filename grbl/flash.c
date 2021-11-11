#include <stm32f4xx_hal_flash.h>

#ifdef AVR // basically unused

/* These EEPROM bits have different names on different devices. */
#ifndef EEPE
		#define EEPE  EEWE  //!< EEPROM program/write enable.
		#define EEMPE EEMWE //!< EEPROM master program/write enable.
#endif

/* These two are unfortunately not defined in the device include files. */
#define EEPM1 5 //!< EEPROM Programming Mode Bit 1.
#define EEPM0 4 //!< EEPROM Programming Mode Bit 0.

/* Define to reduce code size. */
#define EEPROM_IGNORE_SELFPROG //!< Remove SPM flag polling.
 
 #endif

unsigned char flash_get_char(unsigned int addr)
{
    unsigned char* pc = (unsigned char*) addr;    
	return *pc;
}


void flash_put_char(unsigned int addr, unsigned char new_value)
{
    HAL_FLASH_Lock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, new_value);    
    HAL_FLASH_Unlock();
}


void memcpy_to_flash_with_checksum(unsigned int destination, char *source, unsigned int size)
{
    unsigned char checksum = 0;

    HAL_FLASH_Unlock();
    do {
        int blocksize = ((size/8)>0) ? FLASH_TYPEPROGRAM_DOUBLEWORD 
            : ((size/4)>0) ? FLASH_TYPEPROGRAM_WORD
            : ((size/2)>0) ? FLASH_TYPEPROGRAM_HALFWORD
            : FLASH_TYPEPROGRAM_BYTE;

        if (size == 0) break;
        
        for (int j=0; j < blocksize; j++) {
            checksum = (checksum << 1) | (checksum >> 7);
            checksum += *(source+j);
        }
        HAL_FLASH_Program(blocksize, destination, *source);
        source += blocksize;
        size -= blocksize;
    } while (size > 0);
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,destination,checksum);
    HAL_FLASH_Lock();
}


int memcpy_from_flash_with_checksum(char *destination, unsigned int source, unsigned int size)
{
	unsigned char data, checksum = 0;
    for (; size > 0; size--) {
        data = flash_get_char(source++);
        checksum = (checksum << 1) | (checksum >> 7);
        checksum += data;
        *(destination++) = data;
    }
    return(checksum == flash_get_char(source));
}


// end of file
