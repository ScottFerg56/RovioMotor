#include <Arduino.h>
#include <Wire.h>
#include "FLogger.h"
#include "DriveMotor.h"
#include "Head.h"
#include "NavLights.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "LEDAnimator.h"

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
NavLights NavLites = NavLights(EntityID_NavLights, "Nav Lights");
Entity* entities[6] = { &LeftMotor, &RightMotor, &RearMotor, &HeadX, &NavLites, nullptr };

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
        NavLites.Init();
    }
    
    Bot() : Domain(true, entities) {}
};

Bot Rovio;

// our Mac Address = {0xE8, 0x9F, 0x6D, 0x32, 0xDE, 0x2C}

uint8_t ctrlMacAddress[] = {0xE8, 0x9F, 0x6D, 0x22, 0x02, 0xEC};

void setup()
{
    FLogger::setLogLevel(FLOG_DEBUG);
    Serial.begin(115200);
    delay(3000);    // for Serial

    flogi("started");

    Rovio.Init(ctrlMacAddress);
    Rovio.Setup();
    delay(1000);
    flogi("completed");
}

void loop()
{
    LeftMotor.Loop();
    RightMotor.Loop();
    RearMotor.Loop();
    HeadX.Loop();
    NavLites.Loop();
    Rovio.ProcessChanges(nullptr);
}
