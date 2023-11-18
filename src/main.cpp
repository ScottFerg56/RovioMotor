#include <Arduino.h>
#include <Wire.h>
//#include <Adafruit_MotorShield.h>
//#include <QuadEncoder.h>
#include "DriveMotor.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

#include "Packet.h"

unsigned long timeMotorLast = 0;
unsigned long timeStatusLast = 0;

DriveMotor Motors[3];

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

uint8_t broadcastAddress[] = {0xE8, 0x9F, 0x6D, 0x22, 0x02, 0xEC};

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS)
        Serial.println("Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *pData, int len)
{
    if (len == 0 || (len & 1) != 0)
    {
        Serial.println("invalid data packet length");
        return;
    }
    while (len > 0)
    {
        int8_t entity = (*pData & 0xF0) >> 4;
        int8_t property = *pData++ & 0x0F;
        int8_t value = *pData++;
        len -= 2;
        switch (entity)
        {
        case Entities_LeftMotor:
        case Entities_RightMotor:
        case Entities_RearMotor:
            Motors[entity].SetProperty((MotorProperties)property, value);
            break;

        case Entities_AllMotors:
            for (int8_t e = Entities_LeftMotor; e <= Entities_RearMotor; e++)
                Motors[e-Entities_LeftMotor].SetProperty((MotorProperties)property, value);
            break;
        
        default:    // invalid enity
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

    // Right Motor has encoder on opposite side from other motors, so encoder pins must be reversed WRT motor polarity
    Motors[Entities_LeftMotor ].Init(4, 14, A4);
    Motors[Entities_RightMotor].Init(3, A5, 32);
    Motors[Entities_RearMotor ].Init(1, A2, A3);

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

        for (int8_t e = Entities_LeftMotor; e <= Entities_RearMotor; e++)
            Motors[e-Entities_LeftMotor].Loop(dmsec);
    }
    dmsec = msec - timeStatusLast;
    if (dmsec >= 500)
    {
        timeStatusLast = msec;
        int8_t packet[12];
        int8_t* p = packet;
        for (int8_t e = Entities_LeftMotor; e <= Entities_RearMotor; e++)
        {
            if (Motors[e-Entities_LeftMotor].RPMChanged)
            {
                Motors[e-Entities_LeftMotor].RPMChanged = false;
                *p++ = e << 4 | MotorProperties_RPM;
                *p++ = Motors[e-Entities_LeftMotor].RPM;
            }
            if (Motors[e-Entities_LeftMotor].PowerChanged)
            {
                Motors[e-Entities_LeftMotor].PowerChanged = false;
                *p++ = e << 4 | MotorProperties_Power;
                *p++ = Motors[e-Entities_LeftMotor].Power;
            }
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
