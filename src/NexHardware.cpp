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
 * @change remove secondary .read() call from sendCmd to avoid 
 * dropping events
 * 
 **/


#include "NexHardware.h"
#include "NexTouch.h"


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

const uint32_t Nextion::baudRates[]{2400, 4800, 9600, 19200, 31250, 38400, 57600, 115200, 230400, 250000, 256000, 512000, 921600};

// queued events and size
static uint8_t _nextion_queued_events[][2] =
{
    {NEX_RET_EVENT_NEXTION_STARTUP,     6},
    {NEX_RET_EVENT_TOUCH_HEAD,          7},
    {NEX_RET_CURRENT_PAGE_ID_HEAD,      5},
    {NEX_RET_EVENT_POSITION_HEAD,       9},
    {NEX_RET_EVENT_SLEEP_POSITION_HEAD, 9},
    {NEX_RET_AUTOMATIC_SLEEP,           4},
    {NEX_RET_AUTOMATIC_WAKE_UP,         4},
    {NEX_RET_EVENT_NEXTION_READY,       1},
    {NEX_RET_START_SD_UPGRADE,          1},
    {0xFF,                              0}  // end of list
};

/**
 * 
 * @brief Take in serial data and store it into buffer for processing
 * 
 * @return whether or not there is data to parse afterward
 */
bool Nextion::readSerialData()
{
    bool returnVal = false; // assume no data to parse
    while(m_nexSerial->available())
    {
        //while there is data available, put it into a read buffer
        byte newData = m_nexSerial->read(); 
        RX_buffer[++RX_ind] = newData;
        if (NEX_END_TRANSMISSION_VALUE == newData)
        {
            endTransCnt++;
            if(endTransCnt == NEX_END_TRANSMISSION_LENGTH)
            {
                // we have the full transmission ender
                returnVal = true; // get ready to parse afterward
                endTransCnt = 0; // reset message
                RX_ind = 0; // reset location in message
            }
        }
    }
    return returnVal;
}

/**
 * @brief Process the recieved message and handle any new events
 * 
 */
void parseReceivedMessage(NexTouch *nex_listen_list[])
{
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
                if(!isEmpty_nexQueueCommands())
                {
                    nexQueuedCommand event = dequeue_nexQueueCommands();
                    if (event.failCB!=nullptr)
                    {
                        event.failCB();
                    }
                }
            }
            break;
        }
        case NEX_RET_CMD_FINISHED_OK:
        {
            nexQueuedCommand event = dequeue_nexQueueCommands();
            if (RX_buffer[0] != event.successReturnCode && event.failCB!=nullptr)
            {
                // got wrong code back, call failure callback
                event.failCB();
                // otherwise, it succeeded and we do nothing
            }
            break;
        }
        case NEX_RET_SERIAL_BUFFER_OVERFLOW: // buffer overflow event
        {
            if (0xFF == RX_buffer[1] && 0xFF == RX_buffer[2] && 0xFF == RX_buffer[3])
            {
                if(nextionBufferOverflowCallback!=nullptr)
                {
                    nextioNBufferOverflowCallback(); // new, but should have an option
                }
            }
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
            if(!isEmpty_nexQueueCommands())
            {
                nexQueuedCommand event = dequeue_nexQueueCommands();
                if (RX_buffer[0] != event.successReturnCode && event.failCB!=nullptr)
                {
                    // got wrong code back, call failure callback
                    event.failCB();
                }
                else
                {
                    // we have correct return code, now return the string
                    if (event.getterCallback.strCB != nullptr)
                    {
                        // there exists a string callback to send to
                        event.getterCallback.strCB(RX_buffer[1], RX_ind - 3);
                            // adjust buffer start and length to account for pre/post fixes
                    }
                }
            }
            break;
        }
        case NEX_RET_NUMBER_HEAD: // got a number response
        {
            if(!isEmpty_nexQueueCommands())
            {
                nexQueuedCommand event = dequeue_nexQueueCommands();
                if (RX_buffer[0] != event.successReturnCode && event.failCB!=nullptr)
                {
                    // got wrong code back, call failure callback
                    event.failCB();
                    // otherwise, it succeeded and we do nothing
                }
                else
                {
                    // correct return code, now return the number through callback
                    if (event.getterCallback.numCB != nullptr)
                    {
                        int32_t number = ((uint32_t)RX_buffer[4] << 24) | ((uint32_t)RX_buffer[3] << 16) | 
                                        ((uint32_t)RX_buffer[2] << 8) | (RX_buffer[1]);
                        event.getterCallback->numCB(number);
                    }
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
                        
                    touchCoordinateCallback(((int16_t)__buffer[2] << 8) | (__buffer[1]), ((int16_t)__buffer[4] << 8) | (__buffer[3]),__buffer[5]);
                }
                else if(RX_buffer[0] == NEX_RET_EVENT_SLEEP_POSITION_HEAD && touchCoordinateCallback!=nullptr)
                {
                        
                    touchEventInSleepModeCallback(((int16_t)__buffer[2] << 8) | (__buffer[1]), ((int16_t)__buffer[4] << 8) | (__buffer[3]),__buffer[5]);
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
            dbSerialPrint("Bad serial command, header: ");
            dbSerialPrint(RX_buffer[0]);
            dbSerialPrintln();

            // assorted invalid instruction return
            if(!isEmpty_nexQueueCommands())
            {
                nexQueuedCommand event = dequeue_nexQueueCommands();
                if (event.failCB!=nullptr)
                {
                    event.failCB();
                }
            }
            break;              
        }
    }; 
}

/** handle internal command queue **/
bool Nextion::enqueue_nexQueueCommands(nexQueuedCommand event)
{
    eventQ[Qback] = event;
    Qback = (Qback+1) % CMD_QUEUE_SIZE;
    return (Qback != Qfront); // if head/tail at same spot, we overflowed
}

nexQueuedCommand Nextion::dequeue_nexQueueCommands(void)
{
    nexQueuedCommand temp = eventQ[Qfront]; // pull from front
    Qfront = (Qfront+1) % CMD_QUEUE_SIZE;
    return temp;
}

bool Nextion::isEmpty_nexQueueCommands(void)
{
    return (Qfront == Qback); // at same spot, nothing to remove
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

/*
 * Receive unt32_t data. 
 * 
 * @param number - save uint32_t data. 
 * @param timeout - set timeout time. 
 *
 * @retval true - success. 
 * @retval false - failed.
 *
 */
bool Nextion::recvRetNumber(uint32_t *number, size_t timeout)
{
    bool ret = false;
    uint8_t temp[8] = {0};

    if (!number)
    {
        goto __return;
    }

    ReadQueuedEvents();
    if (sizeof(temp) != readBytes(temp, sizeof(temp),timeout))
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_NUMBER_HEAD
        && temp[5] == 0xFF
        && temp[6] == 0xFF
        && temp[7] == 0xFF
        )
    {
        *number = ((uint32_t)temp[4] << 24) | ((uint32_t)temp[3] << 16) | ((uint32_t)temp[2] << 8) | (temp[1]);
        ret = true;
    }

__return:

    if (ret) 
    {
        dbSerialPrint("recvRetNumber: ");
        dbSerialPrintln(*number);
    }
    else
    {
        dbSerialPrintln("recvRetNumber err");
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
    bool ret = false;
    uint8_t temp[8] = {0};

    if (!number)
    {
        goto __return;
    }

    ReadQueuedEvents();
    if (sizeof(temp) != readBytes(temp, sizeof(temp), timeout))
    {
        goto __return;
    }

    if (temp[0] == NEX_RET_NUMBER_HEAD
        && temp[5] == 0xFF
        && temp[6] == 0xFF
        && temp[7] == 0xFF
        )
    {
        *number = ((int32_t)temp[4] << 24) | ((int32_t)temp[3] << 16) | ((int32_t)temp[2] << 8) | (temp[1]);
        ret = true;
    }

__return:

    if (ret) 
    {
        dbSerialPrint("recvRetNumber :");
        dbSerialPrintln(*number);
    }
    else
    {
        dbSerialPrintln("recvRetNumber err");
    }
    
    return ret;
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
    str = "";
    bool ret{false};
    bool str_start_flag {!start_flag};
    uint8_t cnt_0xff = 0;
    uint8_t c = 0;
    ReadQueuedEvents();
    uint32_t start{millis()};
//    size_t avail{(size_t)m_nexSerial->available()};
    while(ret == false && (millis()-start)<timeout)
    {
        while (m_nexSerial->available())
        {
            c = m_nexSerial->read();
            if (str_start_flag)
            {
                if (0xFF == c)
                {
                    cnt_0xff++;                    
                    if (cnt_0xff >= 3)
                    {
                        ret = true;
                        break;
                    }
                }
                else
                {
                    str += (char)c;
                }
            }
            else if (NEX_RET_STRING_HEAD == c)
            {
                str_start_flag = true;
            }
            yield();
        }
        delayMicroseconds(20);
        yield();
    }
    dbSerialPrint("recvRetString[");
    dbSerialPrint(str.length());
    dbSerialPrint(",");
    dbSerialPrint(str);
    dbSerialPrintln("]");

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
    ReadQueuedEvents();
    // empty in buffer for clean responce
    /** below code causes lost events!!!
     * ReadQueuedEvents() should handle it well enough?
    while (m_nexSerial->available())
    {
        m_nexSerial->read();
    }
    */
    
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

size_t Nextion::readBytes(uint8_t* buffer, size_t size, size_t timeout)
{
    uint32_t start{millis()};
    size_t avail{(size_t)m_nexSerial->available()};
    while(size>avail && (millis()-start)<timeout)
    {
        delayMicroseconds(10);
        yield();
        avail=m_nexSerial->available();
    }
    
    size_t read=min(size,avail);
    for(size_t i{read}; i;--i)
    {
        *buffer=m_nexSerial->read();
        ++buffer;
    }
    return read;
}

bool Nextion::recvCommand(const uint8_t command, size_t timeout)
{
    bool ret = false;
    uint8_t temp[4] = {0};
    ReadQueuedEvents();
    size_t bytesRead = readBytes((uint8_t *)temp, sizeof(temp), timeout);
    if (sizeof(temp) != bytesRead)
    {
        dbSerialPrint("recv command timeout: ");

        ret = false;
    }
    else
    {
        if (temp[0] == command
            && temp[1] == 0xFF
            && temp[2] == 0xFF
            && temp[3] == 0xFF
            )
        {
            ret = true;
        }
        else
        {
            dbSerialPrint("recv command err value: ");
            dbSerialPrintln(temp[0]);   
        }
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

void Nextion::nexLoop()
{
    if(readSerialData()) // read the data in
    {
        parseReceivedMessage(m_nex_listen_list); // if we got stuff, decode
    }
}
