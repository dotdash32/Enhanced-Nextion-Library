/**
 * @file NexQueues.h
 * @author Josh deWitt (https://github.com/dotdash32)
 * @brief Queue interface for NexHardware non-blocking/non-dropping
 * @version 0.1
 * @date 2022-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <Arduino.h>

#include "NexConfig.h"


// struct to store stored responses
struct nexResponses
{
    uint8_t RX_buf[RX_2ND_BUF_SIZE] = {0};
    size_t  RX_ind = 0;
};

typedef void (*failureCallback) (uint8_t returnCode);
typedef void (*numberCallback) (int32_t returnNum);
typedef void (*stringCallback) (char *buf, uint16_t len);
enum CommandTypes {
    CT_command, CT_number, CT_stringHead, CT_stringHeadless
};
struct nexQueuedCommand
{
    uint8_t successReturnCode; // what the ideal message should be
    union // if it needs a return value, store the value here
    {
        numberCallback numCB;
        stringCallback strCB;
    };
    failureCallback failCB; // if we get the wrong message header
    unsigned long expiration_time; // when should we assume it's timed out?
    CommandTypes cmdType; // what kind of command is it?
    nexResponses *response; // if storing response, where to?
    
};

// this has to be below the struct definitions
#include "NexHardware.h"


class NexEventQueue
{
public:

// constructor
NexEventQueue();

// destructor
// ~NexEventQueue();

/**
 * @brief Add an event to the Queued Commands
 * 
 * @param event - command response template
 * @return true - success
 * @return false - circular buffer overflow!
 */
bool enqueue(nexQueuedCommand event, size_t *saveSpot = nullptr);
bool enqueuePtr(nexQueuedCommand *event, size_t *saveSpot = nullptr);

/**
 * @brief Get the head of the event queue
 * 
 * 
 * @return event struct
 * 
 */
nexQueuedCommand dequeue(void);
nexQueuedCommand* dequeuePtr(void);

/**
 * @brief Are there events in the queue?
 * 
 * @return true - yes, there are events
 * @return false - no events, don't dequeue an empty queue!
 */
bool isEmpty(void);

nexQueuedCommand peek(void);
nexQueuedCommand* peekPtr(void);

bool passedIndex(size_t *saveSpot);

/**
 * @brief Remove any commands from the Queue that have timed out
 * 
 * @return true - operation removed an event, not clear yet
 * @return false - no operation performed, front of queue is clear 
 *                 of expired events
 */
bool clearExpiredCommands(void);

/***************************************/
private:

    nexQueuedCommand eventQ[CMD_QUEUE_SIZE];
    size_t Qread, Qwrite = {0};
};


class NexResponseQueue
{
public:

// constructor
NexResponseQueue();
     
// destructor
// ~NexResponseQueue();

/**
 * @brief Store most recent response data into response queue
 * at event's saved index
 * 
 * @param event - pointer to event
 * @return true - success, aka valid respQSpot in event
 * @return false - failure, couldn't store data
 */
bool storeData(nexQueuedCommand *event, size_t RX_ind, byte *RX_buffer);

/**
 * @brief Get a slot to store a command Response
 * 
 * @return nexResponses* - pointer to response buffer struct
 */
nexResponses* getResponseSlot(void);
/*******************************************/
private:
    nexResponses respQ[RX_2ND_ARR_SIZE]; // array of responses
    size_t Qwrite = 0; // where can we stick the next value?


};
