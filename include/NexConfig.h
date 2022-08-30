/**
 * @file NexConfig.h
 *
 * Options for user can be found here.
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/13
 * @author Jyrki Berg 2/17/2019 (https://github.com/jyberg)
 *
 * @copyright
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * @copyright 2020 Jyrki Berg
 *
 */

#pragma once

/**
 * @addtogroup Configuration
 * @{
 */

/**
 * Define serial communication default baud.
 * it is recommended that do not change defaul baud on Nextion, because it can forgot it on re-start
 * If changing this value check that vakue is the same as factory set default baud in the used display.
 *
*/
#define NEX_SERIAL_DEFAULT_BAUD 9600

/**
 * Define standard (dafault) or fast timeout,  you may use fast timeout in case of baudrate higher than 115200
 *
*/
#define NEX_TIMEOUT_STANDARD
//#define NEX_TIMEOUT_FAST

#ifdef NEX_TIMEOUT_FAST
#define NEX_TIMEOUT_COMMAND   10
#define NEX_TIMEOUT_RETURN    10
#else
#define NEX_TIMEOUT_COMMAND  200
#define NEX_TIMEOUT_RETURN   100
#endif

#define NEX_TIMEOUT_TRANSPARENT_DATA_MODE 400

// use component names (enabled), or arrays p[PID].b[CID] (commented out)
#define OBJECTS_USE_COMP_NAMES

/**
 * Serial Recieving Buffer Sizes
 *
 * RX_BUFFER_SIZE is the primary buffer that is read into directly after
 * SerialX.read.  It needs to be at least 72 (for comok), but should be larger.
 *
 * CMD_QUEUE_SIZE is the depth of sent Event Queue.  It keeps track of sent events
 * and how to respond in code
 *
 * RX_2ND_BUF_SIZE is the size of the secondary buffer (in an array).  It is used
 * to track responses to respond during blocking loops
 *
 * RX_2ND_ARR_SIZE is the number of entries in the FIFO-like array to store
 * returned messages. (soft limited to 127 spots)
 */
#ifndef NEX_RX_BUFFER_SIZE
  #define NEX_RX_BUFFER_SIZE      128 // primary data buffer length
#endif

#ifndef NEX_CMD_QUEUE_SIZE
  #define NEX_CMD_QUEUE_SIZE      8 // depth of sent event queue
#endif
#ifndef NEX_RESP_BUF_SIZE
  #define NEX_RESP_BUF_SIZE       NEX_RX_BUFFER_SIZE // width of secondary buffer for responses
#endif
#ifndef NEX_RESP_ARR_SIZE
  #define NEX_RESP_ARR_SIZE       8   // depth of secondary buffer
#endif



/**
 * Define DEBUG_SERIAL_ENABLE to enable debug serial.
 * Comment it to disable debug serial.
 */
//#define DEBUG_SERIAL_ENABLE

/**
 * Define dbSerial for the output of debug messages.
 * it is resonsibility of main program to initialize debug serial port (begin(...)
 */
//#define dbSerial Serial

// Enable Next TFT file upload functionality
//#define NEX_ENABLE_TFT_UPLOAD

// Enable HardwareSerial support by definign NEX_ENABLE_HW_SERIAL
#define NEX_ENABLE_HW_SERIAL

// Enable SoftwareSerial support by definign NEX_ENABLE_SW_SERIAL
// NodeMcu / Esp8266 use Softwareserial if usb port is used for debuging
// NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
#define NEX_ENABLE_SW_SERIAL


#ifdef DEBUG_SERIAL_ENABLE
#define dbSerialPrint(a)    dbSerial.print(a)
#define dbSerialPrintln(a)  dbSerial.println(a)
#define dbSerialBegin(a)    dbSerial.begin(a)
#else
#define dbSerialPrint(a)   do{}while(0)
#define dbSerialPrintln(a) do{}while(0)
#define dbSerialBegin(a)   do{}while(0)
#endif

/**
 * @}
 */
