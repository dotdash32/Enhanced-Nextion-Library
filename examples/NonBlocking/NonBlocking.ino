/**
 * @example CompButton.ino
 * 
 * @par How to Use
 * This example demonstrates both blocking and non-blocking code snippets
 * to set and access various data types inside Nextion Components.
 * 
 * 3 Functions by 3 data types (unsigned num, signed num, string)
 * 1: basic button events - sends simple button events, make sure we don't 
 * drop any of these while doing other things
 * 2: Setters - send data MCU -> Nextion, and ensure it makes it there 
 * consisntently and without interfering
 * 3: Getters - retrieve data Nextion -> MCU, ensuring alignment and good
 * return data.
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
#define USE_NON_BLOCKING

// print success messages?
#define PRINT_NB_SUCCESS

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
#define getter_busy_pin1     2


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
NexText     t1          (next,  0, 17, "t1",         &p0);
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
void reqGetter_callback(void* ptr);
void genSuccess_callback(NexObject *obj);
void genFailure_callback(uint8_t returnCode, NexObject *obj);
void number_callback(int32_t num, NexObject *obj);
void string_callback(String str, NexObject *obj);



// setter variables
bool uNum, sNum, sStr = false;  // send updates for these?
unsigned long uNum_time, sNum_time, sStr_time = 0; // last update?

void setup()
{
    pinMode(callback_busy_pin1, OUTPUT);
    pinMode(setter_busy_pin1, OUTPUT);
    pinMode(getter_busy_pin1, OUTPUT);

    digitalWrite(callback_busy_pin1, HIGH);
    Serial.begin(9600);

    // Initialize Nextion connection with selected baud in this case 115200
    if(!next->nexInit(115200))
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
    n0.attachPop(reqGetter_callback, &n0);
    n1.attachPop(reqGetter_callback, &n1);
    t1.attachPop(reqGetter_callback, &t1);
    

    digitalWrite(callback_busy_pin1, LOW);
    digitalWrite(setter_busy_pin1, LOW);    
    digitalWrite(getter_busy_pin1, LOW);
    Serial.println("Ready!");

    // un attached functions: passes "field" param directly through
    next->getStr("connect", string_callback, genFailure_callback, false);
    next->setNum("bkcmd", 3, genSuccess_callback, genFailure_callback);
}

void loop()
{
    next->nexLoop(nex_listen_list);

    // check setters and timing
    checkSetters();
}

void genSuccess_callback(NexObject *obj)
{
    digitalWrite(callback_busy_pin1, HIGH);
    #ifdef PRINT_NB_SUCCESS
        if (nullptr == obj)
        {
            Serial.print("Generic");
        }
        else // we have a calling obj!
        {
            String name;
            obj->getObjInfo(name);
            Serial.print(name);
        }

        Serial.print(" command succeeded!");
        Serial.println();
    #endif /* PRINT_NB_SUCCESS */
    delayMicroseconds(10);
    digitalWrite(callback_busy_pin1, LOW);
}

void genFailure_callback(uint8_t returnCode, NexObject *obj)
{
    digitalWrite(callback_busy_pin1, HIGH);
    if (nullptr == obj)
    {
        Serial.print("Generic");
    } else if (&x0 == obj) {
        Serial.print("Unsigned Number set");
    } else if (&x1 == obj) {
        Serial.print("Signed Number set");
    } else if (&t0 == obj) {
        Serial.print("String set");
    } else if (&n0 == obj) {
        Serial.print("Unsigned Number get");
    } else if (&n1 == obj) {
        Serial.print("Signed Number get");
    } else if (&t1 == obj) {
        Serial.print("String get");
    }

    Serial.print(" command failed with return code: ");
    Serial.print(returnCode, HEX);
    Serial.println();
    digitalWrite(callback_busy_pin1, LOW);
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
    delayMicroseconds(10);
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
            if(!x0.setValue(millis()/100))
            {
                genFailure_callback(0,&x0);
            }
        #else  
            x0.setNum("val",millis()/100,
                      genSuccess_callback,genFailure_callback);
        #endif
        digitalWrite(setter_busy_pin1, LOW);
    }

    if(sNum && (millis() - sNum_time > setterFreq)) {
        sNum_time = millis();
        // send update
        digitalWrite(setter_busy_pin1, HIGH);
        #ifndef USE_NON_BLOCKING
            if(!x1.setValue( (int32_t) -millis()/100))
            {
                genFailure_callback(0,&x1);
            }
        #else  
            x1.setNum("val", (int32_t) - millis() /100,
                      genSuccess_callback, genFailure_callback);
        #endif
        digitalWrite(setter_busy_pin1, LOW);
    }
    if(sStr && (millis() - sStr_time > setterFreq)) {
        sStr_time = millis();
        // send update
        digitalWrite(setter_busy_pin1, HIGH);
        sprintf(buf, "%02d:%02d.%1d",(int) (millis()/60000),
                (int) (millis()/1000 % 60), (int) (millis()%100)/10);
        #ifndef USE_NON_BLOCKING
            if(!t0.setText(buf))
            {
                genFailure_callback(0,&t0);
            }
        #else  
            t0.setStr("txt",buf,genSuccess_callback,genFailure_callback);
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

void reqGetter_callback(void* ptr)
{
    if (nullptr == ptr)
    {
        //nothing assigned!
        Serial.println("unregistered getter requester button!");
    }
    else if (&n0 == ptr)
    {
        digitalWrite(getter_busy_pin1, HIGH);
        #ifndef USE_NON_BLOCKING
            uint32_t num = 0;
            if(!n0.Get_background_color_bco(&num))
            {
                genFailure_callback(0,&n0);
            }
            number_callback((int32_t) num, &n0);
        #else  
            n0.getNum("bco", number_callback, genFailure_callback);
        #endif
        digitalWrite(getter_busy_pin1, LOW);
    }
    else if (&n1 == ptr)
    {
        digitalWrite(getter_busy_pin1, HIGH);
        #ifndef USE_NON_BLOCKING
            int32_t num = 0;
            if(!n1.getValue(&num))
            {
                genFailure_callback(0,&n1);
            }
            number_callback(num, &n1);
        #else  
            n1.getNum("val", number_callback, genFailure_callback);
        #endif
        digitalWrite(getter_busy_pin1, LOW);
    }
    else if (&t1 == ptr)
    {
        digitalWrite(getter_busy_pin1, HIGH);
        String str = "";
        #ifndef USE_NON_BLOCKING
            if(!t1.getText(str))
            {
                genFailure_callback(0,&t1);
            }
            string_callback(str, &t1);
        #else  
            t1.getStr("txt", string_callback, genFailure_callback);
        #endif
        digitalWrite(getter_busy_pin1, LOW);
    }
}

void number_callback(int32_t num, NexObject *obj)
{
    digitalWrite(callback_busy_pin1, HIGH);
    if(nullptr == obj){
        Serial.println("unregistered number return!");
    } else if (&n0 == obj) {
        Serial.print("Got Unsigned Number: ");
        Serial.println(num);
    } else if (&n1 == obj) {
        Serial.print("Got Unsigned Number: ");
        Serial.println(num);
    }
    digitalWrite(callback_busy_pin1, LOW);
}
void string_callback(String str, NexObject *obj)
{
    digitalWrite(callback_busy_pin1, HIGH);
    if(nullptr == obj) {
        Serial.println("unregister String return: ");
        Serial.println(str); // still print it!
    } else if (&t1 == obj) {
        Serial.print("Got String: ");
        Serial.println(str);
    }
    digitalWrite(callback_busy_pin1, LOW);
}
