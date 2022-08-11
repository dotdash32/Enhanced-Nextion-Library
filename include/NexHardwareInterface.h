/**
 * @file NexHardwareInterface.h
 *
 * The definition of pure virtual functions for Nextion. 
 *
 * @author Jyrki Berg 11/23/2019 (https://github.com/jyberg)
 * 
 * @copyright 2020 Jyrki Berg
 */

#pragma once

#include <WString.h>
#ifdef ESP8266
#include <vector>
#endif


class NexObject;
typedef void (*failureCallback) (uint8_t returnCode, NexObject *obj);
typedef void (*numberCallback) (int32_t returnNum, NexObject *obj);
typedef void (*stringCallback) (String returnStr, NexObject *obj);
typedef void (*successCallback) (NexObject *obj);

/**
 * @addtogroup CoreAPI 
 * @{ 
 */

/**
 * Abstract  Nextion Interface class
 *
 * Provides Nextion iterface functions
 */

class NextionInterface
{
public:

    NextionInterface(){}
    virtual ~NextionInterface(){}


/* Receive unsigned number
*
* @param number - received value
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
virtual bool recvRetNumber(uint32_t *number, size_t timeout) =0;

/* Receive signed number
*
* @param number - received value
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
virtual bool recvRetNumber(int32_t *number, size_t timeout) =0;

/* Receive string
*
* @param string - received value
* @param timeout - set timeout time.
* @param start_flag - is str start flag (0x70) expected, default falue true
*
* @retval true - success.
* @retval false - failed. 
*/
virtual bool recvRetString(String &str, size_t timeout, bool start_flag) =0;

/* Receive string
*
* @param buffer - received value buffer
* @param le - value buffer size
* @param timeout - set timeout time.
* @param start_flag - is str start flag (0x70) expected, default falue true
*
* @retval true - success.
* @retval false - failed. 
*/
virtual bool recvRetString(char *buffer, uint16_t &len, size_t timeout, bool start_flag) =0;

/* Send Command to device
*
* parameter command string
*/
virtual void sendCommand(const char* cmd) =0;

/* Send Raw data to device
*
* parameter raw data buffer
*/
#ifdef ESP8266
virtual void sendRawData(const std::vector<uint8_t> &data) =0;
#endif

/* Send Raw data to device
*
* @param buf - raw data buffer poiter
* @param len - raw data buffer pointer
*/
virtual void sendRawData(const uint8_t *buf, uint16_t len) =0;


/* Send Raw byte to device
*
* parameter raw byte
*/
virtual void sendRawByte(const uint8_t byte) =0;

/* read Bytes from device
 * @brief 
 * 
 * @param buffer - receive buffer
 * @param size  - bytes to read
 * @param timeout  timeout ms
 * @return size_t read bytes can be less that size (timeout case) 
 */
// virtual size_t readBytes(uint8_t* buffer, size_t size, size_t timeout) =0;

/* Receive command
*
* @param command - command to be received / checked
* @param timeout - set timeout time.
*
* @retval true - success.
* @retval false - failed. 
*/
virtual bool recvCommand(const uint8_t command, size_t timeout) =0;

/*
 * Command is executed successfully. 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
virtual bool recvRetCommandFinished(size_t timeout) =0;

/*
 * Transpared data mode setup successfully 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
virtual bool RecvTransparendDataModeReady(size_t timeout) =0;

/*
 * Transpared data mode finished 
 *
 * @param timeout - set timeout time.
 *
 * @retval true - success.
 * @retval false - failed. 
 *
 */
virtual bool RecvTransparendDataModeFinished(size_t timeout) =0;

/**
 * current baud value
 * 
 * 
 * @return current baud value
 */
virtual uint32_t GetCurrentBaud() =0;


/**** non-blocking getters/setter ***/

/**
 * @brief Set a String parameter (string input)
 * 
 * @param field - which field/command section to write to
 * @param newText - new textual value to write to parameter
 * @param succCB - success callback
 * @param obj - object that called the function
 * @param failCallback - failure callback
 * @param timeout - how long before assuming no connection
 * @return true - message sent successfully
 * @return false - queue collision!
 */
virtual bool setStr(String field, String newText, successCallback succCB, failureCallback failCallback, NexObject *obj, uint32_t timeout) =0;

/**
 * @brief Set a Number parameter (signed)
 * 
 * Basically all numbers in the Nextion are signed, or are limited to within
 * the positive bounds of a int32_t.  If you REALLY need an unsigned number,
 * make it a uint32_t and cast it to an int32_t before feeding it into this 
 * function.
 * 
 * @param field - which field/command section to write to
 * @param num - number to send
 * @param succCB - success callback
 * @param obj - object that called the function
 * @param failCallback - failure callback
 * @param timeout - how long before assuming no connection
 * @return true - message sent successfully
 * @return false - queue collision!
 */
virtual bool setNum(String field, int32_t num, successCallback succCB, failureCallback failCallback, NexObject *obj, uint32_t timeout) =0;

/**
 * @brief Get a string value
 * 
 * @param field - which field/command section to read from
 * @param retCallback - return callback, sends data to main program
 * @param obj - object that called the function
 * @param failCallback - failure callback
 * @param timeout - how long before assuming no connection
 * @return true - message sent successfully
 * @return false - queue collision!
 */
virtual bool getStr(String field, stringCallback retCallback, failureCallback failCallback, bool start_flag, NexObject *obj, uint32_t timeout) =0;

/**
 * @brief Get a Number value
 * 
 * @param field - which field/command section to read from
 * @param retCallback - return callback, sends data to main program
 * @param obj - object that called the function
 * @param failCallback - failure callback
 * @param timeout - how long before assuming no connection
 * @return true - message sent successfully
 * @return false - queue collision!
 */
virtual bool getNum(String field, numberCallback retCallback, failureCallback failCallback, NexObject *obj, uint32_t timeout) =0;

/**
 * @brief Send Command, non-blocking with callbacks
 * 
 * @param command - cmd to send to the device
 * @param returnCode - first byte expected back when command works
 * @param succCB - success callback
 *        !! WARNING !! won't get called if there is a more generic callback
 *          response for that specific command code
 * @param obj - object that called the function
 * @param failCallback - failure callback
 * @param timeout - how long before assuming no connection
 * @return true - message sent successfully
 * @return false - queue collision! 
 */
virtual bool nbSendCmd(String command, uint8_t returnCode, successCallback succCB, failureCallback failCallback, NexObject *obj, uint32_t timeout) =0;

};

/**
 * @} 
 */
