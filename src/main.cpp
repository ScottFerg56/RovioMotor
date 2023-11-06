//#define KNOBS 1
#define PLOT 1

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include <Adafruit_seesaw.h>
//#include <seesaw_neopixel.h>
#include <QuadEncoder.h>
#include "DriveMotor.h"
#ifdef KNOBS
#include "ScaledKnob.h"
bool DoKnobs = true;
#endif
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

#include "Packet.h"

#ifdef PLOT
bool DoPlot = true;
#endif
unsigned long timeMotorLast = 0;
unsigned long timeStatusLast = 0;

// Right Motor has encoder on opposite side from other motors, so encoder pins must be reversed WRT motor polarity
Encoders LeftMotorEncoder(32,14);
Encoders RightMotorEncoder(A4,A5);
Encoders RearMotorEncoder(A2,A3);

#ifdef KNOBS
Adafruit_seesaw SeeSaw;
seesaw_NeoPixel SSPixel = seesaw_NeoPixel(4, 18, NEO_GRB + NEO_KHZ800);
#endif

void dump(const char* name, float v)
{
  Serial.print(name);
  Serial.print(":");
  Serial.println(v, 3);
}

#ifdef PLOT
void plot(const char* name, float v)
{
  if (!DoPlot)
    return;
  Serial.print(">");
  Serial.print(name);
  //Serial.print(":");
  //Serial.print(timeMotorLast);
  Serial.print(":");
  Serial.println(v, 3);
}
#endif

float Map(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

#ifdef KNOBS
ScaledKnob KnobKp(0, 12, 0, 50, 0.1);
ScaledKnob KnobLeft( 1, 14, -100, 100, 5);
ScaledKnob KnobRight(2, 17, -100, 100, 5);
ScaledKnob KnobRear( 3,  9, -100, 100, 5);
#endif

Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
DriveMotor LeftMotor;
DriveMotor RightMotor;
DriveMotor RearMotor;
int8_t lastLeftMotorRPM;
int8_t lastRightMotorRPM;
int8_t lastRearMotorRPM;

uint8_t broadcastAddress[] = {0xE8, 0x9F, 0x6D, 0x22, 0x02, 0xEC};

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS)
        Serial.println("Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    if (len == 0 || (len & 1) != 0)
    {
        Serial.println("invalid data packet length");
        return;
    }
    for (size_t i = 0; i < len; i += 2)
    {
        switch ((PacketCmds)incomingData[i*2])
        {
        case CmdLeftMotorGoal:
            LeftMotor.SpeedGoal = (int8_t)incomingData[i*2+1];
            break;
        case CmdRightMotorGoal:
            RightMotor.SpeedGoal = (int8_t)incomingData[i*2+1];
            break;
        case CmdRearMotorGoal:
            RearMotor.SpeedGoal = (int8_t)incomingData[i*2+1];
            break;
        }
    }
}
 
void setup()
{
    timeMotorLast = millis();
    Serial.begin(115200);
    delay(1000);    // for Serial

    Serial.println("In setup");
    #ifdef KNOBS
    Serial.println("KNOBS");
    #endif
    #ifdef PLOT
    Serial.println("PLOT");
    #endif

    #ifdef KNOBS
    if (!SeeSaw.begin(0x49) || !SSPixel.begin(0x49))
    {
        Serial.println("Couldn't find seesaw on default address");
        while(1) delay(10);
    }

    SSPixel.setBrightness(50);
#endif

    MotorShield.begin();
    LeftMotor.Init( MotorShield.getMotor(4), &LeftMotorEncoder);
    RightMotor.Init(MotorShield.getMotor(3), &RightMotorEncoder);
    RearMotor.Init( MotorShield.getMotor(1), &RearMotorEncoder);

#ifdef KNOBS
  KnobKp.Init(&SeeSaw, &SSPixel, RearMotor.Kp);
  KnobLeft.Init(&SeeSaw, &SSPixel, LeftMotor.SpeedGoal);
  KnobRight.Init(&SeeSaw, &SSPixel, RightMotor.SpeedGoal);
  KnobRear.Init(&SeeSaw, &SSPixel, RearMotor.SpeedGoal);

  KnobKp.SetColor(0, 0, 128);
  KnobLeft.SetColor(0, 0, 128);
  KnobRight.SetColor(0, 0, 128);
  KnobRear.SetColor(0, 0, 128);
#endif

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        while(1);
    }

    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        while(1);
    }
    
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("Setup complete");
}

void loop()
{
    unsigned long msec = millis();
    unsigned long dmsec = msec - timeMotorLast;
    if (dmsec >= 100)
    { 
        timeMotorLast = msec;

#ifdef KNOBS
        if (DoKnobs)
        {
        KnobKp.Sample();
        KnobLeft.Sample();
        KnobRight.Sample();
        KnobRear.Sample();

        if (KnobKp.Pressed() == ScaledKnob::Presses::Press)
            KnobKp.SetValue(0);
        if (KnobLeft.Pressed() == ScaledKnob::Presses::Press)
            KnobLeft.SetValue(0);
        if (KnobRight.Pressed() == ScaledKnob::Presses::Press)
            KnobRight.SetValue(0);
        if (KnobRear.Pressed() == ScaledKnob::Presses::Press)
            KnobRear.SetValue(0);

        LeftMotor.SpeedGoal = KnobLeft.GetValue();
        LeftMotor.Kp = KnobKp.GetValue();
        RightMotor.SpeedGoal = KnobRight.GetValue();
        RightMotor.Kp = KnobKp.GetValue();
        RearMotor.SpeedGoal = KnobRear.GetValue();
        RearMotor.Kp = KnobKp.GetValue();
        }
#endif

        LeftMotor.Loop(dmsec);
        RightMotor.Loop(dmsec);
        RearMotor.Loop(dmsec);

#ifdef PLOT
        plot("LeftGoal", LeftMotor.SpeedGoal);
        plot("RightGoal", RightMotor.SpeedGoal);
        plot("RearGoal", RearMotor.SpeedGoal);
        //plot("LeftCount", LeftMotor.Count);
        //plot("RightCount", RightMotor.Count);
        //plot("RearCount", RearMotor.Count);
        plot("LeftRPM", LeftMotor.RPM);
        plot("RightRPM", RightMotor.RPM);
        plot("RearRPM", RearMotor.RPM);
        //plot("Kp", RearMotor.Kp);
        //plot("Ki", RearMotor.Ki);
        //plot("Kd", RearMotor.Kd);
        //plot("Error", RearMotor.Error);
        //plot("ErrorSum", RearMotor.ErrorSum);
        //plot("LeftOutput", LeftMotor.Output);
        //plot("RightOutput", RightMotor.Output);
        //plot("RearOutput", RearMotor.Output);
        //plot("LeftPower", LeftMotor.Power);
        //plot("RightPower", RightMotor.Power);
        //plot("RearPower", RearMotor.Power);
#endif
    }
    dmsec = msec - timeStatusLast;
    if (dmsec >= 1000)
    {
        timeStatusLast = msec;
        int8_t packet[6];
        int8_t* p = packet;
        if (LeftMotor.RPM != lastLeftMotorRPM)
        {
            lastLeftMotorRPM = LeftMotor.RPM;
            *p++ = CmdLeftMotorRPM;
            *p++ = lastLeftMotorRPM;
        }
        if (RightMotor.RPM != lastRightMotorRPM)
        {
            lastRightMotorRPM = RightMotor.RPM;
            *p++ = CmdRightMotorRPM;
            *p++ = lastRightMotorRPM;
        }
        if (RearMotor.RPM != lastRearMotorRPM)
        {
            lastRearMotorRPM = RearMotor.RPM;
            *p++ = CmdRearMotorRPM;
            *p++ = lastRearMotorRPM;
        }
        int len = p-packet;
        if (len > 0)
        {
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &packet, len);
            if (result != ESP_OK)
                Serial.println("Error sending the data");
        }
    }
}
