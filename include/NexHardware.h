/**
 * @file NexHardware.h
 *
 * The definition of base API for using Nextion. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/11
 * @author Jyrki Berg 2/24/2019 (https://github.com/jyberg)
 * 
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * 
 * @copyright 2020 Jyrki Berg (https://github.com/jyberg)
 * 
 * @copyright 2020 Jyrki Berg
 *
 */

#pragma once

#include <Arduino.h>


#include "NexConfig.h"
#include "NexHardwareInterface.h"
#include "NexTouch.h"

#include <HardwareSerial.h>

#ifdef NEX_ENABLE_SW_SERIAL
#include <SoftwareSerial.h>
#endif

class NexTouch;


struct nexQueuedEvent
{
    uint8_t event_data[10];
    nexQueuedEvent *m_next{nullptr};
};

typedef (void *failureCallback) (uint8_t returnCode);
typedef (void *numberCallback) (int32_t returnNum);
typedef (void *stringCallback) (char *buf, uint8_t len);
#define CMD_QUEUE_SIZE              8 // how many events can we store?
struct nexQueuedCommand
{
    uint8_t successReturnCode; // what the ideal message should be
    union getterCallback // if it needs a return value, store the value here
    {
        numberCallback numCB;
        stringCallback strCB;
    };
    failureCallback failCB; // if we get the wrong message header
    size_t timeout; // how long until we assume its dead?
    
};

/**
 * @addtogroup CoreAPI 
 * @{ 
 */

/**
 * Nextion connection class
 * 
 * Provides Nextion connection instance either for harware serial or software serial
 * Harware serial can be disable/eabled using NEX_ENABLE_HW_SERIAL define
 * software serial can be disable/eabled using NEX_ENABLE_SW_SERIAL define
 *
 * Note: NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
 * 
 */
class Nextion:public NextionInterface
{
private: // methods

    static const uint32_t baudRates[]; // Nextion supported bauds

    Nextion()=delete;

/**
 * test connection to nextion
 * 
 * @return true if success, false for failure. 
 */
bool connect();

/**
 * resolve baud
 * 
 * @param baud - resolved baud rate
 * 
 * @return true if success, false for failure. 
 */
bool findBaud(uint32_t &baud);

enum serialType {HW, SW, HW_USBCON};

    const serialType m_nexSerialType; 
    Stream *m_nexSerial;
    uint32_t m_baud;
    NexTouch *m_nex_listen_list[];

    nexQueuedEvent *m_queuedEvents{nullptr};


/**
 * @brief Move Serial data from hardware to software buffer
 * 
 * @return true - message finished, ready to parse
 * @return false - no end of message, nothing else to do this loop
 */
bool readSerialData(void);

/**
 * @brief Handle complete message from Nextion
 * 
 * takes any message/event and triggers appropriate response
 * (usually a callback)
 * 
 * @param nex_listen_list - list of pointers to touch event listeners
 */
void parseReceivedMessage(NexTouch *nex_listen_list[]);


/**** Handle internal Queue of Commands and expected responses ***/
/**
 * @brief Add an event to the Queued Commands
 * 
 * @param event - command response template
 * @return true - success
 * @return false - circular buffer overflow!
 */
bool Nextion::enqueue_nexQueueCommands(nexQueuedCommand event);

/**
 * @brief Get the head of the event queue
 * 
 * 
 * @return event struct
 * 
 */
nexQueuedCommand Nextion::dequeue_nexQueueCommands(void);

/**
 * @brief Are there events in the queue?
 * 
 * @return true - yes, there are events
 * @return false - no events, don't dequeue an empty queue!
 */
bool Nextion::isEmpty_nexQueueCommands(void)



public:

/**
 * Nextion connection
 * 
 * @param nexSerial - used serial port instance
 */
Nextion(HardwareSerial &nexSerial);

/**
 * Nextion connection instance
 * 
 * Creates Nextion connection instance
 * NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
 *
 * @param nexSerial - used serial port instance
 *
 *  @retval Nextion instace pointer
 */
static Nextion* GetInstance(HardwareSerial &nexSerial);

#ifdef NEX_ENABLE_SW_SERIAL
/**
 * Nextion connection
 * 
 * NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
 *
 * @param nexSerial - used serial port instance
 */
Nextion(SoftwareSerial &nexSerial);

/**
 * Nextion connection instance
 * 
 * Creates Nextion connection instance
 * NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
 *
 * @param nexSerial - used serial port instance
 *
 *  @retval Nextion instace pointer
 */
static Nextion* GetInstance(SoftwareSerial &nexSerial);
#endif

#ifdef USBCON
/**
 * Nextion connection
 * 
 * NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
 *
 * @param nexSerial - used serial port instance
 */
Nextion(Serial_ &nexSerial);

/**
 * Nextion connection instance
 * 
 * Creates Nextion connection instance
 * NodeMcu board pin numbers not match with Esp8266 pin numbers use NodeMcu Pin number definitions (pins_arduino.h)
 *
 * @param nexSerial - used serial port instance
 *
 *  @retval Nextion instace pointer
 */
static Nextion* GetInstance(Serial_ &nexSerial);
#endif


virtual ~Nextion();

/**
 * Nextion Startup callback function
 * Returned when Nextion has started or reset
 */
 void (*nextionStartupCallback)();
// std::function<void()> nextionStartupCallback;

/**
 * Nextion Buffer Overflow callback function
 * Returned when Nextion looses commands when buffer is full
 */
void (*nextioNBufferOverflowCallback)();

/**
 * Current Page ID callback function
 * The device returns this data after receiving “sendme” instruction)
 * 
 *  uint8_t pageId
 */
 void (*currentPageIdCallback)(uint8_t);
// std::function<void(uint8_t)> currentPageIdCallback;

/**
 * Touch Coordinate callback function
 * When the system variable “sendxy” is 1, return this data at TouchEvent occurring
 * 
 * uint16_t x
 * uint16_t y
 * uint8_t TouchEvent
 * 
 * Definition of TouchEvent: Press Event 0x01, Release Event 0X00 
 */
 void (*touchCoordinateCallback)(uint16_t,uint16_t,uint8_t);
// std::function<void(uint16_t,uint16_t,uint8_t)>  touchCoordinateCallback;

/**
 * Touch Event in sleep mode callback function
 * When the device enters sleep mode, return this data at TouchEvent occurring
 * 
 * uint16_t x
 * uint16_t y
 * uint8_t TouchEvent
 * 
 * Definition of TouchEvent: Press Event 0x01, Release Event 0X00 
 */
 void (*touchEventInSleepModeCallback)(uint16_t,uint16_t,uint8_t);
// std::function<void(uint16_t,uint16_t,uint8_t)> touchEventInSleepModeCallback;

/**
 * Device automatically enters into sleep mode callback function
 * Only when the device automatically enters into sleep mode will return this data.
 * If execute serial command “sleep = 1” to enter into sleep mode, it will not return this data.
 */
 void (*automaticSleepCallback)();
// std::function<void()> automaticSleepCallback;

/**
 * Device automatically wake up callback function
 * Only when the device automatically wake up will return this data.
 * If execute serial command “sleep=0” to wake up, it will not return this data. 
 */
 void (*automaticWakeUpCallback)();
// std::function<void()> automaticWakeUpCallback;

/**
 * Nextion Ready callback function
 * Returned when Nextion has powered up and is now initialized successfully
 */
 void (*nextionReadyCallback)();
// std::function<void()> nextionReadyCallback;

/**
 * Start SD card upgrade callback function
 * This data is sent after the device power on and detect SD card, and then enter upgrade interface
 */
 void (*startSdUpgradeCallback)();
// std::function<void()> startSdUpgradeCallback;

bool Nextion::prepRetNumber(uint8_t returnCode, numberCallback retCallback, 
                            failureCallback failCallback, size_t timeout);
bool Nextion::prepRetString(uint8_t returnCode, stringCallback retCallback, 
                            failureCallback failCallback, size_t timeout);
bool Nextion::prepRetCode(uint8_t returnCode, 
                            failureCallback failCallback, size_t timeout);                                                        

/* Receive unsigned number
*
* @param number - received value
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvRetNumber(uint32_t *number, size_t timeout = NEX_TIMEOUT_RETURN) final;

/* Receive signed number
*
* @param number - received value
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvRetNumber(int32_t *number, size_t timeout = NEX_TIMEOUT_RETURN) final;

/* Receive string
*
* @param string - received value
* @param timeout - set timeout time.
* @param start_flag - is str start flag (0x70) expected, default falue true
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvRetString(String &str, size_t timeout = NEX_TIMEOUT_RETURN, bool start_flag = true) final;

/* Receive string
*
* @param buffer - received value buffer
* @param len - value buffer size
* @param timeout - set timeout time.
* @param start_flag - is str start flag (0x70) expected, default falue true
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvRetString(char *buffer, uint16_t &len, size_t timeout = NEX_TIMEOUT_RETURN, bool start_flag = true) final;

/* Send Command to device
*
*  @param cmd - command string
*/
void sendCommand(const char* cmd) final;

/* Send Raw data to device
*
*  @param data - raw data buffer
*/
#ifdef ESP8266
void sendRawData(const std::vector<uint8_t> &data) final;
#endif

/* Send Raw data to device
*
* @param buf - raw data buffer poiter
* @param len - raw data buffer pointer
*/
void sendRawData(const uint8_t *buf, uint16_t len) final;


/* Send Raw byte to device
*
* @param byte - raw byte
*/
void sendRawByte(const uint8_t byte) final;


/* read Bytes from device
 * @brief 
 * 
 * @param buffer - receive buffer
 * @param size  - bytes to read
 * @param timeout  timeout ms
 * @return size_t read bytes can be less that size (timeout case) 
 */
size_t readBytes(uint8_t* buffer, size_t size, size_t timeout = NEX_TIMEOUT_RETURN) final;

/* Receive command
*
* @param command - command to be received / checked
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
bool recvCommand(const uint8_t command, size_t timeout = NEX_TIMEOUT_COMMAND) final;

/*
 * Command is executed successfully. 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool recvRetCommandFinished(size_t timeout = NEX_TIMEOUT_COMMAND) final;

/*
 * Transpared data mode setup successfully 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool RecvTransparendDataModeReady(size_t timeout = NEX_TIMEOUT_TRANSPARENT_DATA_MODE) final;

/*
 * Transpared data mode finished 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
bool RecvTransparendDataModeFinished(size_t timeout = NEX_TIMEOUT_COMMAND) final;

/**
 * Init Nextion connection.
 * 
 * @param baud - baud value: (2400, 4800, 9600, 19200, 31250, 38400, 57600, 115200, 230400, 250000, 256000, 512000, 921600)
 * 
 * @return true if success, false for failure. 
 */
bool nexInit(const uint32_t baud = NEX_SERIAL_DEFAULT_BAUD);

/**
 * current baud value
 * 
 * 
 * @return current baud value
 */
uint32_t GetCurrentBaud() final;

/**
 * Listen touch event and calling callbacks attached before.
 * 
 * Supports push and pop at present. 
 *
 * @param nex_listen_list - index to Nextion Components list. 
 * @return none. 
 *
 * @warning This function must be called repeatedly to response touch events
 *  from Nextion touch panel. Actually, you should place it in your loop function. 
 */
void nexLoop(NexTouch *nex_listen_list[]);
};

/**
 * @}
 */
