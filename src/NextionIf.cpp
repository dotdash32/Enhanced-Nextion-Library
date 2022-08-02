/**
 * @file NextionIf.cpp
 *
 * Implementation of class NextionIf
 *
 * @author Jyrki Berg 11/23/2019 (https://github.com/jyberg)
 * 
 * @copyright 2020 Jyrki Berg
 * 
 **/

#include "NextionIf.h"
#include "NexHardware.h"


NextionIf::NextionIf(Nextion *nextion):m_nextion{nextion}
{}

NextionIf::~NextionIf()
{}


bool NextionIf::recvRetNumber(uint32_t *number, size_t timeout)
{
    return m_nextion->recvRetNumber(number, timeout);
}

bool NextionIf::recvRetNumber(int32_t *number, size_t timeout)
{
    return m_nextion->recvRetNumber(number, timeout);
}

bool NextionIf::recvRetString(String &str, size_t timeout, bool start_flag)
{
    return m_nextion->recvRetString(str, timeout, start_flag);
}

bool NextionIf::recvRetString(char *buffer, uint16_t &len, size_t timeout, bool start_flag)
{
    return m_nextion->recvRetString(buffer, len, timeout, start_flag);
}

void NextionIf::sendCommand(const char* cmd)
{
    return m_nextion->sendCommand(cmd);
}

#ifdef ESP8266
void NextionIf::sendRawData(const std::vector<uint8_t> &data)
{
    return m_nextion->sendRawData(data);
}
#endif

void NextionIf::sendRawData(const uint8_t *buf, uint16_t len)
{
    return m_nextion->sendRawData(buf, len);
}

void NextionIf::sendRawByte(const uint8_t byte)
{
    return m_nextion->sendRawByte(byte);
}

// size_t NextionIf::readBytes(uint8_t* buffer, size_t size, size_t timeout)
// {
//     return m_nextion->readBytes(buffer, size, timeout);
// }


bool NextionIf::recvCommand(const uint8_t command, size_t timeout)
{
    return m_nextion->recvCommand(command, timeout);
}

bool NextionIf::recvRetCommandFinished(size_t timeout)
{
    return m_nextion->recvRetCommandFinished(timeout);
}

bool NextionIf::RecvTransparendDataModeReady(size_t timeout)
{
    return m_nextion->RecvTransparendDataModeReady(timeout);
}

bool NextionIf::RecvTransparendDataModeFinished(size_t timeout)
{
    return m_nextion->RecvTransparendDataModeFinished(timeout);
}

uint32_t NextionIf::GetCurrentBaud()
{
    return m_nextion->GetCurrentBaud();
}
