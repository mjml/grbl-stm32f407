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

#include "grbl.h"
#include "serial.stm32.h"
#include "usbd_cdc_if.h"


extern USBD_HandleTypeDef hUsbDeviceFS;

// STM32 uses multi-buffering so that the microcontroller's USB stack can send entire buffers asynchronously.
struct rxbuf_t rxbuf[RX_RING_SIZE];
int rxhead=0; // lines consumed at head
int rxtail=0; // lines produced at tail
int rxblocked=false;  // indicator that rx is blocked (rate-limiting) -- to be checked after parsing an rx block
int rxoverflow=false; // indicator that the entire rxbuf ring overflowed -- this is a critical error
int curtx;

// The USBD_CDC stack's line coding structure for baud, datalen, parity, stop, etc
USBD_CDC_LineCodingTypeDef linecoding = {
  38400,
  0x00,
  0x00,
  0x08
};

void init_txbufs () 
{
  curtx=0;
  for (int i=0; i<2; i++) {
    txbuf[i].pos = 0;
    txbuf[i].avail = 1;
  }
}

void init_rxbufs () 
{
  for (int i=0; i<RX_RING_SIZE; i++) {
    rxbuf[i].mode = 0;
    rxbuf[i].len = 0;
    rxbuf[i].pos = 0;
  }
  rxhead = rxtail = 0;
  rxblocked = false;
  rxoverflow = false;
}


void flip_txbuf() 
{
  curtx = (curtx+1) % 2;
}

uint8_t is_delim (uint8_t b) {
  return b=='\n' || b=='\r' || b==0;
}

uint8_t* find_delim (uint8_t* buf, int len) 
{
  for (int i=0; i < len; i++) {
    if (is_delim(buf[i])) {
      return buf+i;
    }
  }
  return 0;

}


// Prepares rxhead so that the CDC read buffer can be set, returns 0 if successful or 1 if blocked.
// Will set rxblock accordingly.
uint8_t get_unused_rxbufs() 
{
  int usedblocks = (rxhead>=rxtail) ? rxhead-rxtail : rxhead-rxtail+RX_RING_SIZE;
  return RX_RING_SIZE - usedblocks;
}

uint8_t get_all_rxbufs_incomplete() 
{
  for (int i=rxtail; i != rxhead; i=(i+1)%RX_RING_SIZE) {
    if (rxbuf[i].mode != RXBUF_MODE_INCOMPLETE) {
      return false;
    }
  }
  return true;
}

uint8_t rxbuf_is_readable (struct rxbuf_t* pbuf)
{
  return pbuf->mode == RXBUF_MODE_COMPLETE || pbuf->mode == RXBUF_MODE_PARTIAL;
}

// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  return RX_BUFFER_SIZE - rxbuf[rxtail].pos;
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
  return rxbuf[rxtail].pos;
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  return txbuf[curtx].pos;
}


void serial_init()
{
  if (USBD_Init (&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {  
    printf("USBD_Init Failed.\n");
    return;
  }

  if (USBD_RegisterClass (&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
    printf("USBD_RegisterClass Failed.\n");
    return;
  }

  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
    printf("USBD_CDC_RegisterInterface Failed.\n");
    return;
  }

  if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
    printf("USBD_Start Failed.\n");
    return;
  }
  printf("USBD Initialization Complete.\n");
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
  uint16_t pos = txbuf[curtx].pos;
  uint8_t next_pos = txbuf[curtx].pos + 1;
  
  txbuf[curtx].buf[pos] = data;
  txbuf[curtx].pos = next_pos;

  // STM32's usb stack needs a bit of a priming here:
  // The problem here is that the realtime codes like $J don't end in a carriage return and we need some extra state-tracking to flush them immediately.
  if (next_pos == TX_BUFFER_SIZE || data == '\n') {
    // make the current buffer unavailable
    txbuf[curtx].avail = 0;

    // queue the finished buffer for sending
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, txbuf[curtx].buf, txbuf[curtx].pos);

    // check if we can flip to the alternate buffer, then do so
    int alttx = (curtx+1)%2;
    while (!txbuf[alttx].avail) {}
    flip_txbuf();

  }
}


// Data Register Empty Interrupt handler
#ifdef AVR
ISR(SERIAL_UDRE)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

  // Send a byte from the buffer
  UDR0 = serial_tx_buffer[tail];

  // Update tail position
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}
#endif


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint16_t pos = rxbuf[rxtail].pos;
  if (pos == rxbuf[rxtail].len) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = rxbuf[rxtail].buf[pos];
    rxbuf[rxtail].pos++;
    return data;
  }
}

#ifdef AVR
ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint8_t next_head;

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}
#endif

void serial_reset_read_buffer()
{
  init_rxbufs();
}


//// STM32 CDC Callbacks ////
int8_t serial_init_cb(void) 
{
  // just reset all counters and scalar fields
  init_txbufs();
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rxbuf[rxhead].buf + rxbuf[rxhead].len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, txbuf[curtx].buf, TX_BUFFER_SIZE);
  return USBD_OK;
}

int8_t serial_deinit_cb(void)
{
  // basically do the same reset as in the init_cb()
  init_txbufs();
  return USBD_OK;
}

int8_t serial_control_cb(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    USBD_SetupReqTypedef* req = NULL;

  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

      break;

    case CDC_SET_COMM_FEATURE:

      break;

    case CDC_GET_COMM_FEATURE:

      break;

    case CDC_CLEAR_COMM_FEATURE:

      break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
      linecoding.bitrate = (uint32_t)( pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24) );
      linecoding.format      = pbuf[4];
      linecoding.paritytype  = pbuf[5];
      linecoding.datatype    = pbuf[6];
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(linecoding.bitrate);
      pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
      pbuf[4] = (uint8_t)(linecoding.format);
      pbuf[5] = (uint8_t)(linecoding.paritytype);
      pbuf[6] = (uint8_t)(linecoding.datatype);
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      req = (USBD_SetupReqTypedef*)pbuf;
      if (req->wValue & 0x0001) {
        host_connected=1;
      }
      break;

    case CDC_SEND_BREAK:
      // TODO: Send EOF? This should cue the host terminal to close its connection.
      break;

    default:
      break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */

}

/**
 * Note: Incoming packets at FS have a packet size of 64. This is the largest value you should see in *Len.
 */
int8_t serial_recv_cb(uint8_t* pbuf, uint32_t *Len)
{
  // Update the tail rxblock's fields
  rxbuf[rxhead].len += *Len;

  // Update the status of the head rxbuf
  uint8_t complete = is_delim(pbuf[*Len-1]);
  uint8_t* contains_delim = find_delim(pbuf, *Len);
  uint8_t partial = rxbuf[rxhead].mode == RXBUF_MODE_PARTIAL;
  if (complete) {
    rxbuf[rxhead].mode = RXBUF_MODE_COMPLETE;
  } else if (partial || contains_delim) {
    rxbuf[rxhead].mode = RXBUF_MODE_PARTIAL;
  } else {
    rxbuf[rxhead].mode = RXBUF_MODE_INCOMPLETE;
  }

  /* Pick off realtime characters from the incoming USB stream.   
     In the STM32 implementation, we act on these here in the interrupt callback just as in AVR,
       but we leave them in the buffer and implement a preprocessor in protocol.c that will remove them before execution.
  */
  for (int i=0; i < *Len; i++) {
    uint8_t data = pbuf[*Len];
    switch(data) {
      case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
      case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
      case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
      case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
      default :
        if (data > 0x7F) { // Real-time control characters are extended ACSII only.
          switch(data) {
            case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
            case CMD_JOG_CANCEL:   
              if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
                system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
              }
              break; 
            #ifdef DEBUG
              case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
            #endif
            case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
            case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
            case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
            case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
            case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
            case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
            case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
            case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
            case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
            case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
            case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
            case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
            case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
            case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
            case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
            #ifdef ENABLE_M7
              case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
            #endif
          }
          // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
        }     
    }
  }

  // Determine whether we have room for the next packet or if we are blocked
  int remaining_space = RX_BUFFER_SIZE - rxbuf[rxhead].len;
  int newhead = (rxhead+1) % RX_RING_SIZE;
  if (complete || (remaining_space < CDC_DATA_FS_MAX_PACKET_SIZE)) {
    // The next packet would go in a new input block, so check if we have one
    int unused_blocks = get_unused_rxbufs();
    if (unused_blocks > 0) {
      if (rxbuf[newhead].mode != RXBUF_MODE_UNUSED) {
        rxblocked = true;
        // check for overflow (ie: all rxblocks are incomplete)
        if (get_all_rxbufs_incomplete()) {
          // This is a critical error condition (an overflow of the entire rxbuf_t ring) 
          // We will likely have to re-init the entire rxbuf ring, but do this out of the main loop.
          rxoverflow = true;
          return USBD_OK;
        }
      } else {
        rxblocked = false;
        rxhead = newhead;
      }
    } else {
      rxblocked = true;
    }
  } 

  // If we're blocked, don't put the hardware back in receive mode until the parser has consumed some of the consuming/incomplete lines
  if (!rxblocked) {
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rxbuf[rxhead].buf + rxbuf[rxhead].len);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
  
  return USBD_OK;
}

int8_t serial_txcplt_cb(uint8_t *pbuf, uint32_t *Len, uint8_t epnum)
{
  int oldtx = 0;
  if (pbuf == txbuf[1].buf) {
    oldtx = 1;
  }
  txbuf[oldtx].pos = 0;
  txbuf[oldtx].avail = 1;

  return USBD_OK;
}