/**
 * @example CompButton.ino
 * 
 * @par How to Use
 * This example demonstrates both blocking and non-blocking code snippets
 * to set and access various data types inside Nextion Components.
 *
 * @author  Josh DeWitt (https://github.com/dotash32)
 * @date    2022/8/5
 *
 * @copyright 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * 
 * 
 */

#include "Arduino.h"
#include <SoftwareSerial.h>

#include "Nextion.h"

#include "Arduino.h"


/***** Options ******/

// always print the timing on basic button events?
// #define BUTTON_ALWAYS_PRINT

// Use non-blocking functions
// #define USE_NON_BLOCKING

// timing values
#define setterFreq  90 // [ms] how often to SEND update to fields

// match these settings to the Nextion
#define buttonFreq  100 // [ms] how often basic button will send evemt
#define getterFreq  80  // [ms] how often getter will request update

/*** end options ***/

#ifdef ESP8266
// esp8266 / NodeMCU software serial ports
SoftwareSerial mySerial(D2, D1); // RX, TX
#else
// SoftwareSerial mySerial(3,2); // RX, TX
#define NextionSerial Serial4 // for Teensy 3.6 testing
#endif

// Logic Analyzer busy pins
#define callback_busy_pin1   0
#define setter_busy_pin1     1


// Declare Nextion Instance
Nextion *next = Nextion::GetInstance(NextionSerial);
// create a page
NexPage p0(next, 0, "page0");


NexDSButton sndBasic    (next,  0,  1, "sndBasic",   &p0);
NexTimer    btnTmr      (next,  0,  2, "btnTmr",     &p0);

// time setter functions
NexDSButton ReqUNum     (next,  0,  3, "ReqUNum",    &p0);
NexNumber   x0          (next,  0,  4, "x0",         &p0); // positive time
NexDSButton ReqSNum     (next,  0,  5, "ReqSNum",    &p0);
NexNumber   x1          (next,  0, 10, "x1",         &p0); // negative time
NexDSButton ReqStr      (next,  0,  6, "ReqStr",     &p0);
NexText     t0          (next,  0,  7, "t0",         &p0); // text time

// random num/str getters
NexDSButton ReqUNumGet  (next,  0,  8, "ReqUNumGet", &p0);
NexButton   unGen       (next,  0,  9, "unGen",      &p0);
NexNumber   n0          (next,  0, 11, "n0",         &p0);
NexDSButton ReqSNumGet  (next,  0, 12, "ReqSNumGet", &p0);
NexButton   snGen       (next,  0, 13, "snGen",      &p0);
NexNumber   n1          (next,  0, 14, "n1",         &p0);
NexDSButton ReqStrGet   (next,  0, 15, "ReqStrGet",  &p0);
NexButton   strGen      (next,  0, 16, "strGen",     &p0);
NexNumber   t1          (next,  0, 17, "t1",         &p0);
NexVariable letters     (next,  0, 18, "letters",    &p0); // lookup table
NexTimer    getTmr      (next,  0, 20, "getTmr",     &p0);

// reset on both sides
NexButton   reset       (next,  0, 19, "reset",      &p0);




NexTouch *nex_listen_list[] = 
{
    &sndBasic,
    &ReqUNum,
    // &x0, // set only, no listen
    &ReqSNum,
    // &x1, // set only, no listen
    &ReqStr,
    // &t0, // set only, no listen
    &ReqUNumGet,
    // &unGen, //on Nextion side only
    &n0,
    &ReqSNumGet,
    // &snGen, //on Nextion side only
    &n1,
    &ReqStrGet,
    // &strGen, //on Nextion side only
    &t1,
    &reset,
    NULL
};

// call backs/ function protos
void basicBtn_callback(void *ptr);
void reset_callback(void *ptr);
void checkSetters(void);
void requester_callback(void* ptr);



// setter variables
bool uNum, sNum, sStr = false;  // send updates for these?
unsigned long uNum_time, sNum_time, sStr_time = 0; // last update?

void setup()
{
    pinMode(callback_busy_pin1, OUTPUT);
    pinMode(setter_busy_pin1, OUTPUT);

    digitalWrite(callback_busy_pin1, HIGH);
    Serial.begin(9600);

    // Initialize Nextion connection with selected baud in this case 115200
    if(!next->nexInit(9600))
    {
        Serial.println("nextion init fails"); 
    }
    digitalWrite(callback_busy_pin1, LOW);


    // attach callbacks!
    sndBasic.attachPop(basicBtn_callback, &sndBasic);
    sndBasic.attachPush(basicBtn_callback, &sndBasic);
    reset.attachPop(reset_callback, &reset);

    // ReqUNum.attachPop(reqUNum_callback, &ReqUNum);
    ReqUNum.attachPop(requester_callback, &ReqUNum);
    ReqSNum.attachPop(requester_callback, &ReqSNum);
    ReqStr.attachPop(requester_callback, &ReqStr);

    digitalWrite(callback_busy_pin1, LOW);
    digitalWrite(setter_busy_pin1, LOW);    
    Serial.println("Ready!");


}

void loop()
{
    next->nexLoop(nex_listen_list);
    // Serial.println("in loop");

    // check setters and timing
    checkSetters();
}

void basicBtn_callback(void *ptr)
{
    digitalWrite(callback_busy_pin1, HIGH);
    static uint16_t interval = 0;
    static uint16_t lastPush = 0;
    interval = millis() - lastPush;
    lastPush = millis();
    if (interval > 2*buttonFreq) {
        Serial.print("missed push: ");
        Serial.print(interval);
        Serial.println();
    } else {
        #ifdef BUTTON_ALWAYS_PRINT
            Serial.print("timer push: ");
            Serial.print(interval);
            Serial.println();
        #endif
    }
    digitalWrite(callback_busy_pin1, LOW);
}

void reset_callback(void *ptr)
{
    // reset interval variables!

    // dual state buttons we are tracking
    uNum = false;
    sNum = false;
    sStr = false;
}

void checkSetters(void)
{
    static char buf[10];
    if(uNum && (millis() - uNum_time > setterFreq)) {
        uNum_time = millis();
        // send update
        digitalWrite(setter_busy_pin1, HIGH);
        #ifndef USE_NON_BLOCKING
            x0.setValue(millis()/100);
        #else  
        #endif
        digitalWrite(setter_busy_pin1, LOW);
    }

    if(sNum && (millis() - sNum_time > setterFreq)) {
        sNum_time = millis();
        // send update
        digitalWrite(setter_busy_pin1, HIGH);
        #ifndef USE_NON_BLOCKING
            x1.setValue( (int32_t) -millis()/100);
        #else  
        #endif
        digitalWrite(setter_busy_pin1, LOW);
    }
    if(sStr && (millis() - sStr_time > setterFreq)) {
        sStr_time = millis();
        // send update
        digitalWrite(setter_busy_pin1, HIGH);
        sprintf(buf, "%02d:%02d",millis()/60000, millis()/1000 % 60);
        #ifndef USE_NON_BLOCKING
            t0.setText(buf);
        #else  
        #endif
        digitalWrite(setter_busy_pin1, LOW);
    }
}


void requester_callback(void* ptr)
{
    if(nullptr == ptr)
    {
        Serial.println("unregistered requester button!");
    }
    else if (&ReqUNum == ptr) uNum = !uNum;
    else if (&ReqSNum == ptr) sNum = !sNum;
    else if(&ReqStr == ptr)   sStr = !sStr;
    
}