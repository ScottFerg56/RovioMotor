#include <Arduino.h>
#include <Wire.h>
#include "FLogger.h"
#include "DriveMotor.h"
#include "Head.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

#include "Packet.h"
#include "Domain.h"

//
// NOTE: These Entities might best be declared within the Domain subclass,
// as is done with the Properties in the Entity class.
// But that is causing unexplained catastrophic failure that is possibly
// due to C++ random order of object construction??
// Fortunately the domain and these entities ony need one instance,
// so placing them outside the class works well enough locically.
//
DriveMotor LeftMotor = DriveMotor(EntityID_LeftMotor, "Left Motor");
DriveMotor RightMotor = DriveMotor(EntityID_RightMotor, "Right Motor");
DriveMotor RearMotor = DriveMotor(EntityID_RearMotor, "Rear Motor");
Head HeadX = Head(EntityID_Head, "Head");
Entity* entities[5] = { &LeftMotor, &RightMotor, &RearMotor, &HeadX, nullptr };

class Bot : public Domain
{
public:
    void Setup()
    {
        // Right Motor has encoder on opposite side from other motors, so encoder pins must be reversed WRT motor polarity
        LeftMotor.Init(4, 14, A4);
        RightMotor.Init(3, A5, 32);
        RearMotor.Init(1, A2, A3);
        HeadX.Init(2, 37);
    }
    
    Bot() : Domain(true, entities) {}
};

Bot Rovio;

unsigned long timeMotorLast = 0;
unsigned long timeStatusLast = 0;

uint8_t ctrlMacAddress[] = {0xE8, 0x9F, 0x6D, 0x22, 0x02, 0xEC};

void setup()
{
    FLogger::setLogLevel(FLOG_DEBUG);
    timeMotorLast = millis();
    Serial.begin(115200);
    delay(1000);    // for Serial

    flogi("started");

    Rovio.Init(ctrlMacAddress);
    Rovio.Setup();
    delay(1000);
    flogi("completed");
}

void loop()
{
    unsigned long msec = millis();
    // time slice for processing changes coming in from the controller Domain
    // and reporting Properties we've changed to controller
    unsigned long dmsec = msec - timeStatusLast;
    if (dmsec >= 500)
    {
        timeStatusLast = msec;

        const int len = 10;
        Packet packets[len];     // enough for a few properties
        uint8_t cnt = 0;
        Rovio.ProcessChanges(nullptr);
    }
    // time slice for controlling robot Entities
    dmsec = msec - timeMotorLast;
    if (dmsec >= 100)
    {
        timeMotorLast = msec;

        LeftMotor.Loop(dmsec);
        RightMotor.Loop(dmsec);
        RearMotor.Loop(dmsec);
        HeadX.Loop(dmsec);
    }
}
