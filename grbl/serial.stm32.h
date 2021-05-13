/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef serial_stm32_h
#define serial_stm32_h


#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 251
#endif
#ifndef TX_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define TX_BUFFER_SIZE 124
  #else
    #define TX_BUFFER_SIZE 104
  #endif
#endif

//#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define RX_RING_SIZE 8     // number of "command" data blocks
#define RX_RING_RESERVE 5  // how many free data blocks are required for a read request

// this is the maximum length input command that the client should consider sending in order for the device to safely accept input without overflow
#define RX_MAX_INPUT_LINE  (RX_BUFFER_SIZE - CDC_DATA_MAX_PACKET_SIZE)   

#define SERIAL_NO_DATA 0xff

enum rxbuf_mode {
  RXBUF_MODE_UNUSED,     // block is unused and ready for producing
  RXBUF_MODE_PRODUCING,  // block is being used to write incoming data
  RXBUF_MODE_INCOMPLETE, // block is no longer used to write incoming data, consists of an incomplete line
  RXBUF_MODE_PARTIAL,    // block is no longer used to write incoming data, has complete line(s), but ends with an incomplete line
  RXBUF_MODE_COMPLETE    // block is no longer used to write incoming data, consists only of complete lines
};

struct rxbuf_t {
  uint8_t  mode; // 0:unused 1:producing 2:consuming
  uint16_t pos;
  uint16_t len;  
  uint8_t buf[RX_BUFFER_SIZE];
};


// Circular queue of input lines
extern struct rxbuf_t rxbuf[RX_RING_SIZE];
extern int rxhead; 
extern int rxtail; 
extern int rxblocked;
extern int rxoverflow;

struct txbuf_t {
  uint16_t pos;
  uint8_t  avail;
  uint8_t  pad0;
  uint8_t buf[TX_BUFFER_SIZE];
};

extern struct txbuf_t txbuf[2];

void serial_init();

// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data);

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read();

// Reset and empty data in read buffer. Used by e-stop and reset.
void serial_reset_read_buffer();

uint8_t serial_is_read_overflow();

// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available();

// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count();

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count();


//// STM32 USB CDC Callbacks ////
int8_t serial_init_cb(void);
int8_t serial_deinit_cb(void);
int8_t serial_control_cb(uint8_t cmd, uint8_t* pbuf, uint16_t length);
int8_t serial_recv_cb(uint8_t* pbuf, uint32_t *Len);
int8_t serial_txcplt_cb(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

#endif
