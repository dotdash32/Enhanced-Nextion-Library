/**
 * @file NexQueues.cpp
 * 
 * Implementation of different queues for NexHardware internals
 * 
 * @author Josh deWitt (https://github.com/dotdash32)
 * @brief Ring Buffer queues for sent events and responses
 * @version 0.1
 * @date 2022-08-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "NexQueues.h"

// constructor
NexEventQueue::NexEventQueue()
{
    Qwrite = 0; Qread = 0;
    // array is statically allocated
}

/** handle internal command queue **/
bool NexEventQueue::enqueue(nexQueuedCommand event, size_t* saveSpot)
{
    eventQ[Qwrite % CMD_QUEUE_SIZE] = event;
    if(nullptr != saveSpot)
    {
        *saveSpot = Qwrite; // were we did the insertion
    }
    Qwrite++; // don't wrap!
    
    // if tail is ahead of head, overflowed
    return ((Qwrite+1 %CMD_QUEUE_SIZE) != (Qread %CMD_QUEUE_SIZE)); 
}

// same as above, but a pointer for efficiancy??
// This function is called BEFORE filling the struct with data
// it fills the internal queue directly, via the pointer
bool NexEventQueue::enqueuePtr(nexQueuedCommand *event, size_t *saveSpot)
{
    event = &eventQ[Qwrite % CMD_QUEUE_SIZE]; // export pointer to use
    if(nullptr != saveSpot)
    {
        *saveSpot = Qwrite; // were we did the insertion
    }
    Qwrite++; // don't wrap!
    
    // if tail is ahead of head, overflowed
    return ((Qwrite+1 %CMD_QUEUE_SIZE) != (Qread %CMD_QUEUE_SIZE)); 
}

nexQueuedCommand NexEventQueue::dequeue(void)
{
    nexQueuedCommand temp = eventQ[Qread % CMD_QUEUE_SIZE]; // pull from front
    Qread++;
    return temp;
}
nexQueuedCommand* NexEventQueue::dequeuePtr(void)
{
    nexQueuedCommand *temp = &eventQ[Qread % CMD_QUEUE_SIZE]; // pull from front
    Qread++;
    return temp;
}

bool NexEventQueue::isEmpty(void)
{
    return (Qread == Qwrite); // at same spot, nothing to remove
}

bool NexEventQueue::passedIndex(size_t *saveSpot)
{
    bool returnVal = false; // assume we haven't passed it yet
    // check if we can even do the maths
    if (saveSpot != nullptr)
    {
        // two cases, no wrap and wrap

        // check wrap first
        if (Qwrite < CMD_QUEUE_SIZE && Qread > CMD_QUEUE_SIZE)
        {
            // write index just rolled over, but read has not
            returnVal = ((Qread % CMD_QUEUE_SIZE) > (*saveSpot % CMD_QUEUE_SIZE));
        }
        //no wrap
        if (Qread > *saveSpot)
        {
            returnVal = true; // we passed it!
        }
    }
    return returnVal;
}

nexQueuedCommand NexEventQueue::peek(void)
{
    return eventQ[Qread % CMD_QUEUE_SIZE]; // look, but don't remove
}
nexQueuedCommand* NexEventQueue::peekPtr(void)
{
    return &eventQ[Qread % CMD_QUEUE_SIZE];
}

bool NexEventQueue::clearExpiredCommands(void)
{
    bool returnVal = false; // assume there's nothing to clean up
    if(!isEmpty())
    {
        // only operate on a not empty queue!
        nexQueuedCommand *event = peekPtr();
        if (millis() >= event->expiration_time)
        {
            // event is TOO OLD
            dequeuePtr(); // formally pull it out
            returnVal = true;
            #ifdef LOW_LEVEL_DEBUG
                Serial.println("removed old event! ");
            #endif
        }
    }
    return returnVal;
}



/************** Response Queue **********/

NexResponseQueue::NexResponseQueue()
{
    Qwrite = 0;
}

// put data into response Queue
bool NexResponseQueue::storeData(nexQueuedCommand *event, size_t RX_ind, byte *RX_buffer)
{
    bool returnVal = false;
    if (nullptr != event->response) // have a storage location
    {
        // nexResponses *response = &respQ[event->respSlot]; // get pointer
        event->response->RX_ind = RX_ind;
        memcpy(event->response->RX_buf, RX_buffer, RX_ind);
        returnVal = true;
    }
    return returnVal;
}

nexResponses* NexResponseQueue::getResponseSlot(void)
{
    nexResponses *temp = &respQ[Qwrite % RX_2ND_ARR_SIZE];
    Qwrite++;
    return temp; // pass it out
}