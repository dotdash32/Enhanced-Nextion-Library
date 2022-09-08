/**
 * @file NexHardware.cpp
 *
 * The implementation of base API for using Nextion.
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/11
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
 * @author Josh DeWitt 7/28/2022 (https://github.com/dotdash32)
 *
 **/


#include "NexHardware.h"
#include "NexTouch.h"

// debug helper
// #define LOW_LEVEL_DEBUG


#define NEX_RET_EVENT_NEXTION_STARTUP       (0x00)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_AUTOMATIC_SLEEP             (0x86)
#define NEX_RET_AUTOMATIC_WAKE_UP           (0x87)
#define NEX_RET_EVENT_NEXTION_READY         (0x88)
#define NEX_RET_START_SD_UPGRADE            (0x89)
#define Nex_RET_TRANSPARENT_DATA_FINISHED   (0xFD)
#define Nex_RET_TRANSPARENT_DATA_READY      (0xFE)

#define NEX_RET_INVALID_CMD             (0x00)
#define NEX_RET_CMD_FINISHED_OK         (0x01)
#define NEX_RET_INVALID_COMPONENT_ID    (0x02)
#define NEX_RET_INVALID_PAGE_ID         (0x03)
#define NEX_RET_INVALID_PICTURE_ID      (0x04)
#define NEX_RET_INVALID_FONT_ID         (0x05)
#define NEX_RET_INVALID_FILE_OPERATION  (0x06)
#define NEX_RET_INVALID_CRC             (0x09)
#define NEX_RET_INVALID_BAUD            (0x11)
#define NEX_RET_INVALID_WAVEFORM_ID_OR_CHANNEL_NRO  (0x12)
#define NEX_RET_INVALID_VARIABLE_OR_ATTRIBUTE       (0x1A)
#define NEX_RET_INVALID_VARIABLE_OPERATION          (0x1B)
#define NEX_RET_ASSIGNMENT_FAILED_TO_ASSIGN         (0x1C)
#define NEX_RET_EEPROM_OPERATION_FAILED             (0x1D)
#define NEX_RET_INVALID_QUANTITY_OF_PARAMETERS      (0x1E)
#define NEX_RET_IO_OPERATION_FAILED                 (0x1F)
#define NEX_RET_ESCAPE_CHARACTER_INVALID            (0x20)
#define NEX_RET_VARIABLE_NAME_TOO_LONG              (0x23)
#define NEX_RET_SERIAL_BUFFER_OVERFLOW              (0x24)

#define NEX_END_TRANSMISSION_VALUE                  (0xFF)  // what value ends transmission?
#define NEX_END_TRANSMISSION_LENGTH                 3       // how many of the above to end transmission?


const uint32_t Nextion::baudRates[]{2400, 4800, 9600, 19200, 31250, 38400, 57600, 115200, 230400, 250000, 256000, 512000, 921600};

// buffer things
static byte RX_buffer[RX_BUFFER_SIZE] = {0}; // array to store incoming values in
static size_t RX_ind = 0; // where in the buffer array are we?
static size_t RX_ind_old = 0; // save for parsing
static uint8_t endTransCnt = 0; // how many end of message bytes have we rec'd?

// create queue objects
static NexEventQueue cmdQ = NexEventQueue();
static NexResponseQueue respQ = NexResponseQueue();

/**
 *
 * @brief Take in serial data and store it into buffer for processing
 *
 * @return whether or not there is data to parse afterward
 */
bool Nextion::readSerialData(void)
{
    bool returnVal = false; // assume no data to parse
    while(m_nexSerial->available())
    {
        //while there is data available, put it into a read buffer
        byte newData = m_nexSerial->read();
        #ifdef LOW_LEVEL_DEBUG
            Serial.print(newData,HEX);
            Serial.print(" ");
        #endif /* LOW_LEVEL_DEBUG*/

        RX_buffer[RX_ind++] = newData;
        if (NEX_END_TRANSMISSION_VALUE == newData && RX_ind == 1)
        {
            // we shouldn't get an end transmission at first
            RX_ind--; // back up
        }
        if (NEX_END_TRANSMISSION_VALUE == newData && RX_ind > 1)
        {
            // actually at the end of a message
            static bool inTerminator = false;
            if (NEX_RET_NUMBER_HEAD == RX_buffer[0])
            {
                // if receiving a number, special terminator case
                // "worst case": 0x71 [FF FF FF FF] (FF FF FF) | return -1
                inTerminator = (RX_ind >= 6);
            }
            else
            {
                // standard case, just look for 3 consecutive values
                inTerminator = (NEX_END_TRANSMISSION_VALUE == RX_buffer[RX_ind-2])
                                != (0 == endTransCnt); // logical XOR
            }
            if (inTerminator) // actually at end of message?
            {
                // ensure it is continuous
                endTransCnt++;
                if(endTransCnt == NEX_END_TRANSMISSION_LENGTH)
                {
                    #ifdef LOW_LEVEL_DEBUG
                        Serial.println();
                        Serial.println("        message terminated");
                        Serial.print  ("        length: "); Serial.print(RX_ind);
                        Serial.println();
                    #endif /* LOW_LEVEL_DEBUG*/

                    // we have the full transmission ender
                    returnVal = true; // get ready to parse afterward
                    endTransCnt = 0; // reset message
                    RX_ind_old = RX_ind; // store for safety??
                    RX_ind = 0; // reset location in message
                    parseReceivedMessage(m_nex_listen_list); // if we got stuff, decode
                    return true;
                }
            }
            else
            {
                // previous value was not also a terminator, restart count
                endTransCnt = 0;
            }
        }
        else
        {
            // not a real terminator, reset count
            endTransCnt = 0;
        }
    }
    return returnVal;
}

/**
 * @brief Process the recieved message and handle any new events
 *
 */
void Nextion::parseReceivedMessage(NexTouch *nex_listen_list[])
{
    #ifdef LOW_LEVEL_DEBUG
        Serial.print("in parser ");
        Serial.print(RX_buffer[0], HEX);
        Serial.println();
    #endif /* LOW_LEVEL_DEBUG*/

    switch(RX_buffer[0])
    {
        case NEX_RET_EVENT_NEXTION_STARTUP: // for turn on, initial startup
        {
            if (0x00 == RX_buffer[1] && 0x00 == RX_buffer[2] &&
                0xFF == RX_buffer[3] && 0xFF == RX_buffer[4] && 0xFF == RX_buffer[5])
            {
                if(nextionStartupCallback!=nullptr)
                {
                    nextionStartupCallback();
                }
            }
            else if (0xFF == RX_buffer[1] && 0xFF == RX_buffer[2] && 0xFF == RX_buffer[3])
            {
                // invalid instruction return
                if(!cmdQ.isEmpty())
                {
                    nexQueuedCommand event = cmdQ.dequeue();
                    if (event.failCB!=nullptr)
                    {
                        event.failCB(RX_buffer[0], event.calling_object);
                    }
                }
            }
            break;
        }
        case NEX_RET_CMD_FINISHED_OK:
        {
            if (!cmdQ.isEmpty())
            {
                nexQueuedCommand event = cmdQ.dequeue();
                #ifdef LOW_LEVEL_DEBUG
                    Serial.print("CT: ");
                    Serial.print(event.cmdType);
                    Serial.print(", isEmpty: ");
                    Serial.println(cmdQ.isEmpty());
                #endif /* LOW_LEVEL_DEBUG*/
                if (RX_buffer[0] != event.successReturnCode && event.failCB!=nullptr)
                {
                    // got wrong code back, call failure callback
                    event.failCB(RX_buffer[0], event.calling_object);
                    // otherwise, it succeeded and we do nothing
                }
                else if (RX_buffer[0] == event.successReturnCode && event.succCB != nullptr)
                {
                    // got correct code, call success callback!
                    event.succCB(event.calling_object);
                }
                respQ.storeData(&event, RX_ind_old, RX_buffer);
            }
            break;
        }
        case NEX_RET_SERIAL_BUFFER_OVERFLOW: // buffer overflow event
        {
            // if (0xFF == RX_buffer[1] && 0xFF == RX_buffer[2] && 0xFF == RX_buffer[3])
            // {
            //     if(nextionBufferOverflowCallback!=nullptr)
            //     {
            //         nextioNBufferOverflowCallback(); // new, but should have an option
            //     }
            // }
            break;
        }
        case NEX_RET_EVENT_TOUCH_HEAD: // i.e. a button press
        {
            if (0xFF == RX_buffer[4] && 0xFF == RX_buffer[5] && 0xFF == RX_buffer[6])
            {
                NexTouch::iterate(nex_listen_list, RX_buffer[1], RX_buffer[2], RX_buffer[3]);
            }
            break;
        }
        case NEX_RET_CURRENT_PAGE_ID_HEAD:
        {
            if (0xFF == RX_buffer[2] && 0xFF == RX_buffer[3] && 0xFF == RX_buffer[4])
            {
                if(currentPageIdCallback!=nullptr)
                {
                    currentPageIdCallback(RX_buffer[1]);
                }
            }
            break;
        }
        case NEX_RET_STRING_HEAD: // got a string response!
        {
            if(!cmdQ.isEmpty())
            {
                nexQueuedCommand event = cmdQ.dequeue();
                if (RX_buffer[0] != event.successReturnCode && event.failCB!=nullptr)
                {
                    // got wrong code back, call failure callback
                    event.failCB(RX_buffer[0], event.calling_object);
                }
                else
                {
                    // we have correct return code, now return the string
                    respQ.storeData(&event, RX_ind_old, RX_buffer);
                    if (event.strCB != nullptr)
                    {
                        // there exists a string callback to send to
                        String retStr = String();
                        for (size_t i = 1; i < RX_ind_old-3; i++)
                        {
                            retStr += (char) RX_buffer[i];
                        }
                        event.strCB(retStr, event.calling_object);
                    }
                }
            }
            break;
        }
        case NEX_RET_NUMBER_HEAD: // got a number response
        {
            if(!cmdQ.isEmpty())
            {
                nexQueuedCommand event = cmdQ.dequeue();
                if (RX_buffer[0] != event.successReturnCode && event.failCB!=nullptr)
                {
                    // got wrong code back, call failure callback
                    event.failCB(RX_buffer[0], event.calling_object);
                    // otherwise, it succeeded and we do nothing
                }
                else
                {
                    // correct return code, now return the number through callback
                    if (event.numCB != nullptr)
                    {
                        int32_t number = ((uint32_t)RX_buffer[4] << 24) | ((uint32_t)RX_buffer[3] << 16) |
                                        ((uint32_t)RX_buffer[2] << 8) | (RX_buffer[1]);
                        event.numCB(number, event.calling_object);
                    }
                    respQ.storeData(&event, RX_ind_old, RX_buffer);
                }
            }
            break;
        }
        case NEX_RET_EVENT_POSITION_HEAD:
        case NEX_RET_EVENT_SLEEP_POSITION_HEAD:
        {
            if (0xFF == RX_buffer[6] && 0xFF == RX_buffer[7] && 0xFF == RX_buffer[8])
            {
                if(RX_buffer[0] == NEX_RET_EVENT_POSITION_HEAD && touchCoordinateCallback!=nullptr)
                {

                    touchCoordinateCallback(((int16_t)RX_buffer[2] << 8) | (RX_buffer[1]), ((int16_t)RX_buffer[4] << 8) | (RX_buffer[3]),RX_buffer[5]);
                }
                else if(RX_buffer[0] == NEX_RET_EVENT_SLEEP_POSITION_HEAD && touchCoordinateCallback!=nullptr)
                {

                    touchEventInSleepModeCallback(((int16_t)RX_buffer[2] << 8) | (RX_buffer[1]), ((int16_t)RX_buffer[4] << 8) | (RX_buffer[3]),RX_buffer[5]);
                }
            }
            break;
        }
        case NEX_RET_AUTOMATIC_SLEEP:
        case NEX_RET_AUTOMATIC_WAKE_UP:
        {
            if (0xFF == RX_buffer[1] && 0xFF == RX_buffer[2] && 0xFF == RX_buffer[3])
            {
                if(RX_buffer[0]==NEX_RET_AUTOMATIC_SLEEP && automaticSleepCallback!=nullptr)
                {
                    automaticSleepCallback();
                }
                else if(RX_buffer[0]==NEX_RET_AUTOMATIC_WAKE_UP && automaticWakeUpCallback!=nullptr)
                {
                    automaticWakeUpCallback();
                }
            }
            break;
        }
        case NEX_RET_EVENT_NEXTION_READY:
        {
            if(nextionReadyCallback!=nullptr)
            {
                nextionReadyCallback();
            }
            break;
        }
        case NEX_RET_START_SD_UPGRADE:
        {
            if(startSdUpgradeCallback!=nullptr)
            {
                startSdUpgradeCallback();
            }
            break;
        }


        default: // if we are here, it's probably a bad event
        {
            // could be headless string??

            // assorted invalid instruction return
            if(!cmdQ.isEmpty())
            {
                nexQueuedCommand event = cmdQ.dequeue();
                #ifdef LOW_LEVEL_DEBUG
                    Serial.print("CT: ");
                    Serial.print(event.cmdType);
                    Serial.print(", isEmpty: ");
                    Serial.println(cmdQ.isEmpty());
                #endif /* LOW_LEVEL_DEBUG*/

                if (event.cmdType == CT_stringHeadless)
                {
                    // we expect a string WITHOUT a start header
                    respQ.storeData(&event, RX_ind_old, RX_buffer);
                    if(event.strCB != nullptr)
                    {
                        // there exists a string callback to send to
                        String retStr = String();
                        for (size_t i = 0; i < RX_ind_old-3; i++)
                        {
                            retStr += (char) RX_buffer[i];
                        }
                        event.strCB(retStr, event.calling_object);
                            // adjust buffer start and length to account for post fixes
                    }
                }
                else if (RX_buffer[0] == event.successReturnCode && event.succCB != nullptr)
                {
                    // got correct code, call success callback!
                    // we just don't know what said code is otherwise
                    event.succCB(event.calling_object);
                }
                else if (event.failCB!=nullptr)
                {
                    event.failCB(RX_buffer[0], event.calling_object);
                }
            }

            dbSerialPrint("Bad serial command, header: ");
            dbSerialPrint(RX_buffer[0]);
            dbSerialPrintln();

            break;
        }
    };
}


void Nextion::resetSerialReader(void)
{
    // reset the buffer indices
    RX_ind = 0;
    endTransCnt = 0;

    // clear the queue
    while(!cmdQ.isEmpty())
    {
        cmdQ.dequeuePtr(); // use Ptr version for less return data
    }
}


#ifdef NEX_ENABLE_HW_SERIAL
Nextion::Nextion(HardwareSerial &nexSerial):m_nexSerialType{HW},m_nexSerial{&nexSerial},
    nextionStartupCallback{nullptr},
    nextioNBufferOverflowCallback{nullptr},
    currentPageIdCallback{nullptr},
    touchCoordinateCallback{nullptr},
    touchEventInSleepModeCallback{nullptr},
    automaticSleepCallback{nullptr},
    automaticWakeUpCallback{nullptr},
    nextionReadyCallback{nullptr},
    startSdUpgradeCallback{nullptr}
    {}

Nextion* Nextion::GetInstance(HardwareSerial &nexSerial)
{
    return new Nextion(nexSerial);
}
#endif

#ifdef NEX_ENABLE_SW_SERIAL
Nextion::Nextion(SoftwareSerial &nexSerial):m_nexSerialType{SW},m_nexSerial{&nexSerial},
    nextionStartupCallback{nullptr},
    nextioNBufferOverflowCallback{nullptr},
    currentPageIdCallback{nullptr},
    touchCoordinateCallback{nullptr},
    touchEventInSleepModeCallback{nullptr},
    automaticSleepCallback{nullptr},
    automaticWakeUpCallback{nullptr},
    nextionReadyCallback{nullptr},
    startSdUpgradeCallback{nullptr}
{}

#ifdef USBCON
Nextion::Nextion(Serial_ &nexSerial):m_nexSerialType{HW_USBCON},m_nexSerial{&nexSerial},
    nextionStartupCallback{nullptr},
    nextioNBufferOverflowCallback{nullptr},
    currentPageIdCallback{nullptr},
    touchCoordinateCallback{nullptr},
    touchEventInSleepModeCallback{nullptr},
    automaticSleepCallback{nullptr},
    automaticWakeUpCallback{nullptr},
    nextionReadyCallback{nullptr},
    startSdUpgradeCallback{nullptr}
    {}
Nextion* Nextion::GetInstance(Serial_ &nexSerial)
{
    return new Nextion(nexSerial);
}
#endif

Nextion* Nextion::GetInstance(SoftwareSerial &nexSerial)
{
    return new Nextion(nexSerial);
}
#endif

Nextion::~Nextion()
{}


bool Nextion::connect()
{
    m_nexSerial->flush(); // clear before send at start
    resetSerialReader(); // clear internal registers
    sendCommand("");
    sendCommand("connect");
    String resp;
    recvRetString(resp,NEX_TIMEOUT_RETURN, false);
    if(resp.indexOf("comok") != -1)
    {
        dbSerialPrint("Nextion device details: ");
        dbSerialPrintln(resp);
        return true;
    }
    return false;
}

bool Nextion::findBaud(uint32_t &baud)
{
    for(uint8_t i = 0; i < (sizeof(baudRates)/sizeof(baudRates[0])); i++)
    {
        if (m_nexSerialType==HW)
        {
            ((HardwareSerial*)m_nexSerial)->begin(baudRates[i]);
        }
#ifdef NEX_ENABLE_SW_SERIAL
        if (m_nexSerialType==SW)
        {
            ((SoftwareSerial*)m_nexSerial)->begin(baudRates[i]);
        }
#endif
#ifdef USBCON
        if (m_nexSerialType==HW_USBCON)
        {
            ((Serial_*)m_nexSerial)->begin(baudRates[i]);
        }
#endif
        delay(100);
        if(connect())
        {
            baud = baudRates[i];
            dbSerialPrint("Nextion found baud: ");
            dbSerialPrintln(baud);
            return true;
        }
    }
    return false;
}

bool Nextion::prepRetNumber(numberCallback retCallback, failureCallback failCallback,
                            NexObject *obj, size_t timeout)
{
    nexQueuedCommand event;
    event.successReturnCode = NEX_RET_NUMBER_HEAD;
    event.numCB = retCallback;
    event.failCB = failCallback;
    event.expiration_time = millis() + timeout;
    event.cmdType = CT_number;
    event.response = nullptr; // can't access it!
    event.calling_object = obj;

    // put in queue
    return cmdQ.enqueue(event);
}
bool Nextion::prepRetString(stringCallback retCallback, failureCallback failCallback,
                            NexObject *obj, bool start_flag, size_t timeout)
{
    nexQueuedCommand event;
    event.successReturnCode = NEX_RET_STRING_HEAD;
    event.strCB = retCallback;
    event.failCB = failCallback;
    event.expiration_time = millis() + timeout;
    event.cmdType = start_flag ? CT_stringHead : CT_stringHeadless; //which type of string
    event.response = nullptr;
    event.calling_object = obj;

    // put in queue
    return cmdQ.enqueue(event);
}
bool Nextion::prepRetCode(uint8_t returnCode, successCallback retCallback,
                          failureCallback failCallback, NexObject *obj,
                          size_t timeout)
{
    nexQueuedCommand event ;
    event.successReturnCode = returnCode;
    event.succCB = retCallback; // can do return for code success too
    event.failCB = failCallback;
    event.expiration_time = millis() + timeout;
    event.cmdType = CT_command;
    event.response = nullptr;
    event.calling_object = obj;

    // put in queue
    return cmdQ.enqueue(event);
}

bool Nextion::prepRetNumberBlocking(nexResponses *&respSlot, size_t *saveSpot,
                                    size_t timeout)
{
    nexQueuedCommand event;
    event.successReturnCode = NEX_RET_NUMBER_HEAD;
    event.numCB = nullptr;
    event.failCB = nullptr;
    event.expiration_time = millis() + timeout;
    event.cmdType = CT_number;
    event.calling_object = nullptr; // don't track this now

    // get response Queue spot
    respSlot = respQ.getResponseSlot();
    event.response = respSlot; // save into event

    // put in queue
    return cmdQ.enqueue(event, saveSpot);
}

bool Nextion::prepRetStringBlocking(nexResponses *&respSlot, size_t *saveSpot,
                                    bool start_flag, size_t timeout)
{
    nexQueuedCommand event;
    event.successReturnCode = NEX_RET_STRING_HEAD;
    event.strCB = nullptr;
    event.failCB = nullptr;
    event.expiration_time = millis() + timeout;
    event.cmdType = start_flag ? CT_stringHead : CT_stringHeadless; //which type of string
    event.calling_object = nullptr; // don't track this now

    // get response Queue spot
    respSlot = respQ.getResponseSlot();
    event.response = respSlot; // save into event

    // put in queue
    return cmdQ.enqueue(event, saveSpot);
}

bool Nextion::prepRetCodeBlocking(nexResponses *&respSlot, size_t *saveSpot,
                                  uint8_t returnCode, size_t timeout)
{
    nexQueuedCommand event;
    event.successReturnCode = returnCode;
    event.numCB = nullptr; // defaults to none
    event.failCB = nullptr;
    event.expiration_time = millis() + timeout;
    event.cmdType = CT_command;
    event.calling_object = nullptr; // don't track this now

    // get response Queue spot
    respSlot = respQ.getResponseSlot();
    event.response = respSlot; // save into event

    // put in queue
    return cmdQ.enqueue(event, saveSpot);
}

/*
 * Receive unt32_t data.
 *
 * @param number - save uint32_t data.
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed.
 *
 * blocking code!! for backwards compatibility
 *
 * One caveat: if an event callback triggers another blocking wait,
 * the first blocking wait will fail (wrong data in buffer)
 *
 */
bool Nextion::recvRetNumber(uint32_t *number, size_t timeout)
{
    bool ret = true;

    if (!number)
    {
        dbSerialPrintln("recvRetNumber err, no number pointer");
    }
    else
    {
        nexResponses *respSlot = nullptr;
        size_t saveSpot;
        ret &= prepRetNumberBlocking(respSlot, &saveSpot, timeout); // start the queue
            // if it's false, we failed anyway
        while (!cmdQ.passedIndex(&saveSpot))
        {
            // while there are other events (ahead of us with actual callbacks)
            // just sit here and loop
            nexLoop();
            yield();
        }

        // when we break, our event should be the last one processed
            // probable bug: if we added another thing to queue, above is false
            // propbably fix: uses secondary queue
        if (nullptr != respSlot)
        {
            if (NEX_RET_NUMBER_HEAD == respSlot->RX_buf[0] && respSlot->RX_ind == 8)
            {
                *number = ((uint32_t)respSlot->RX_buf[4] << 24) |
                          ((uint32_t)respSlot->RX_buf[3] << 16) |
                          ((uint32_t)respSlot->RX_buf[2] << 8)  |
                          ((uint32_t)respSlot->RX_buf[1]);
                ret &= true;
            }
            else
            {
                ret = false;
            }
        }
        else // don't have a read location
        {
            ret = false;
        }

        if (ret)
        {
            dbSerialPrint("recvRetNumber: ");
            dbSerialPrintln(*number);
        }
        else
        {
            dbSerialPrintln("recvRetNumber err");
        }

    }
    return ret;
}

/*
 * Receive int32_t data.
 *
 * @param number - save int32_t data.
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed.
 *
 */
bool Nextion::recvRetNumber(int32_t *number, size_t timeout)
{
    bool returnVal = recvRetNumber(reinterpret_cast<uint32_t*>(number), timeout);
    *number = (int32_t) *number; // recast it
    return returnVal;
}

/*
 * Receive string data.
 *
 * @param str - save string data.
 * @param timeout - set timeout time.
 * @param start_flag - is str start flag (0x70) expected, default falue true
 *
 * @retval true - success.
 * @retval false - failed.
 *
 */
bool Nextion::recvRetString(String &str, size_t timeout, bool start_flag)
{
    bool ret = true;
    str = "";

    nexResponses *respSlot = nullptr;
    size_t saveSpot = 0;
    ret &= prepRetStringBlocking(respSlot, &saveSpot, start_flag, timeout); // start the queue
        // if it's false, we failed anyway
    while (!cmdQ.passedIndex(&saveSpot))
    {
        // while there are other events (ahead of us with actual callbacks)
        // just sit here and loop
        nexLoop();
        yield();
    }

    // when we break, our event should be the last one processed
    if(nullptr != respSlot)
    {
        if ((NEX_RET_STRING_HEAD == respSlot->RX_buf[0] || !start_flag) &&
            (respSlot->RX_ind > 4))
        {
            uint16_t index = 0; // assume there is no offset
            if (start_flag)
            {
                // we expect a header, so offset by 1
                index = 1;
            }
            ret &= true;

            for (; index <= (respSlot->RX_ind-4); index++)
            {
                // add the chars to the string
                str += (char) respSlot->RX_buf[index];
            }
            dbSerialPrint("recvRetString[");
            dbSerialPrint(str.length());
            dbSerialPrint(",");
            dbSerialPrint(str);
            dbSerialPrintln("]");
        }
        else
        {
            ret = false; // reading failed!
        }
    }
    else // don't have a place to store/retrieve data
    {
        ret = false;
    }

    return ret;
}

/*
 * Receive string data.
 *
 * @param buffer - save string data.
 * @param len - in buffer len / out saved string len excluding null char.
 * @param timeout - set timeout time.
 * @param start_flag - is str start flag (0x70) expected, default falue true
 *
 *
 * @retval true - success.
 * @retval false - failed.
 *
 */
bool Nextion::recvRetString(char *buffer, uint16_t &len, size_t timeout, bool start_flag)
{
    String temp;
    bool ret = recvRetString(temp,timeout, start_flag);

    if(ret && len)
    {
        len=temp.length()>len?len:temp.length();
        strncpy(buffer,temp.c_str(), len);
    }
    return ret;
}

/*
 * Send command to Nextion.
 *
 * @param cmd - the string of command.
 */
void Nextion::sendCommand(const char* cmd)
{
    #ifdef LOW_LEVEL_DEBUG
        Serial.print("cmd: ");
        Serial.println(cmd);
    #endif /* LOW_LEVEL_DEBUG*/

    m_nexSerial->print(cmd);
    m_nexSerial->write(0xFF);
    m_nexSerial->write(0xFF);
    m_nexSerial->write(0xFF);
}

#ifdef ESP8266
void Nextion::sendRawData(const std::vector<uint8_t> &data)
{
    m_nexSerial->write(data.data(),data.size());
}
#endif

void Nextion::sendRawData(const uint8_t *buf, uint16_t len)
{
    m_nexSerial->write(buf, len);
}

void Nextion::sendRawByte(const uint8_t byte)
{
    m_nexSerial->write(&byte, 1);
}

bool Nextion::recvCommand(const uint8_t command, size_t timeout)
{
    bool ret = true;

    nexResponses *respSlot = nullptr;
    size_t saveSpot;
    ret &= prepRetCodeBlocking(respSlot, &saveSpot, command, timeout); // start the queue
        // if it's false, we failed anyway
    while (!cmdQ.passedIndex(&saveSpot))
    {
        // while there are other events (ahead of us with actual callbacks)
        // just sit here and loop
        nexLoop();
        yield();
    }

    // when we break, our event should be the last one processed
    if (nullptr != respSlot)
    {
        if (command == respSlot->RX_buf[0] && respSlot->RX_ind == 4)
        {
            ret &= true;
        }
        else
        {
            ret = false;
        }
    }
    else // no place to save/retrieve from
    {
        ret = false;
    }

    if (ret)
    {
        dbSerialPrint("recv command: ");
        dbSerialPrintln(command);
    }
    else
    {
        dbSerialPrintln("recv command err");
    }

    return ret;
}

bool Nextion::recvRetCommandFinished(size_t timeout)
{
    bool ret = recvCommand(NEX_RET_CMD_FINISHED_OK, timeout);
    if (ret)
    {
        dbSerialPrintln("recvRetCommandFinished ok");
    }
    else
    {
        dbSerialPrintln("recvRetCommandFinished err");
    }
    return ret;
}

bool Nextion::RecvTransparendDataModeReady(size_t timeout)
{
    dbSerialPrintln("RecvTransparendDataModeReady requested");
    bool ret = recvCommand(Nex_RET_TRANSPARENT_DATA_READY, timeout);
    if (ret)
    {
        dbSerialPrintln("RecvTransparendDataModeReady ok");
    }
    else
    {
        dbSerialPrintln("RecvTransparendDataModeReady err");
    }
    return ret;
}

bool Nextion::RecvTransparendDataModeFinished(size_t timeout)
{
    bool ret = recvCommand(Nex_RET_TRANSPARENT_DATA_FINISHED, timeout);
    if (ret)
    {
        dbSerialPrintln("RecvTransparendDataModeFinished ok");
    }
    else
    {
        dbSerialPrintln("RecvTransparendDataModeFinished err");
    }
    return ret;
}

bool Nextion::nexInit(const uint32_t baud)
{

    m_baud=NEX_SERIAL_DEFAULT_BAUD;
    if (m_nexSerialType==HW)
    {
        // try to connect first with default baud as display may have forgot set baud
        ((HardwareSerial*)m_nexSerial)->begin(m_baud); // default baud, it is recommended that do not change defaul baud on Nextion, because it can forgot it on re-start
        if(!connect())
        {
            if(!findBaud(m_baud))
            {
                ((HardwareSerial*)m_nexSerial)->begin(NEX_SERIAL_DEFAULT_BAUD);
                return false;
            }
        }
        if(baud!=NEX_SERIAL_DEFAULT_BAUD  || baud!=m_baud)
        {
            // change baud to wanted
            char cmd[14];
            sprintf(cmd,"baud=%lu",(unsigned long)baud);
            sendCommand(cmd);
            delay(100);
            ((HardwareSerial*)m_nexSerial)->begin(baud);
            if(!connect())
            {
                return false;
            }
            m_baud=baud;
        }
    }
#ifdef NEX_ENABLE_SW_SERIAL
    if (m_nexSerialType==SW)
    {
        // try to connect first with default baud as daspaly may have forgot set baud
        ((SoftwareSerial*)m_nexSerial)->begin(m_baud); // default baud, it is recommended that do not change defaul baud on Nextion, because it can forgot it on re-start
        if(!connect())
        {
            if(!findBaud(m_baud))
            {
                ((SoftwareSerial*)m_nexSerial)->begin(NEX_SERIAL_DEFAULT_BAUD);
                return false;
            }
        }
        if(baud!=NEX_SERIAL_DEFAULT_BAUD || baud!=m_baud)
        {
            // change baud to wanted
            char cmd[14];
            sprintf(cmd,"baud=%lu",(unsigned long)baud);
            sendCommand(cmd);
            delay(100);
            ((SoftwareSerial*)m_nexSerial)->begin(baud);
            if(!connect())
            {
                return false;
            }
            m_baud=baud;
        }
    }
#endif
    dbSerialPrint("Used Nextion baud: ");
    dbSerialPrintln(m_baud);
    sendCommand("bkcmd=3");
    recvRetCommandFinished();
    sendCommand("page 0");
    bool ret = recvRetCommandFinished();
    return ret;
}

uint32_t Nextion::GetCurrentBaud()
{
    return m_baud;
}

void Nextion::nexLoop(NexTouch *nex_listen_list[])
{
    if (nullptr == m_nex_listen_list)
    {
        // on first run, save the listen list
        m_nex_listen_list = nex_listen_list;
    }
    readSerialData();
    cmdQ.clearExpiredCommands(); // remove any old commands
}

bool Nextion::setStr(String field, String newText, successCallback succCB,
                     failureCallback failCallback, NexObject *obj,
                     uint32_t timeout)
{
    String cmd = field;
    cmd += "=\"";
    cmd += newText; // format command
    cmd += "\""; // add quotes around the value!
    sendCommand(cmd.c_str());
    return prepRetCode(NEX_RET_CMD_FINISHED_OK, succCB, failCallback,
                       obj, timeout);
}

bool Nextion::setNum(String field, int32_t num, successCallback succCB,
                     failureCallback failCallback, NexObject *obj,
                     uint32_t timeout)
{
    String cmd = field;
    cmd += "=0x";
    char buf[9] = {0}; // create a teeny buffer
    itoa(num, buf, HEX);
    if ((strlen(buf) % 2) != 0)
    {
        // not byte aligned!  add a leading zero
        cmd += "0";
    }
    cmd += buf;
    sendCommand(cmd.c_str());
    return prepRetCode(NEX_RET_CMD_FINISHED_OK, succCB, failCallback,
                       obj, timeout);
}

bool Nextion::getStr(String field, stringCallback retCallback,
                     failureCallback failCallback, bool start_flag,
                     NexObject *obj, uint32_t timeout)
{
    sendCommand(field.c_str());
    return prepRetString(retCallback, failCallback, obj,
                                    start_flag, timeout);
}
bool Nextion::getNum(String field, numberCallback retCallback,
                     failureCallback failCallback, NexObject *obj,
                     uint32_t timeout)
{
    sendCommand(field.c_str());
    return prepRetNumber(retCallback, failCallback, obj,
                                    timeout);
}
bool Nextion::nbSendCmd(String command, uint8_t returnCode,
                        successCallback succCB,
                        failureCallback failCallback,
                        NexObject *obj, uint32_t timeout)
{
    sendCommand(command.c_str());
    return prepRetCode(returnCode, succCB, failCallback, obj, timeout);
}
