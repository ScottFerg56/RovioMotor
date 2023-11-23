#include <Arduino.h>
#include <Wire.h>
#include "FLogger.h"
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

uint8_t ctrlMacAddress[] = {0xE8, 0x9F, 0x6D, 0x22, 0x02, 0xEC};

esp_now_peer_info_t peerInfo;

DriveMotor& getMotor(Entities entity)
{
    return Motors[entity - Entities_LeftMotor];
}

void setEntityProperty(Entities entity, Properties property, int16_t value)
{
    switch (entity)
    {
    case Entities_LeftMotor:
    case Entities_RightMotor:
    case Entities_RearMotor:
        getMotor(entity).setProperty(property, value);
        break;
    }
}

int16_t getEntityProperty(Entities entity, Properties property)
{
    switch (entity)
    {
    case Entities_LeftMotor:
    case Entities_RightMotor:
    case Entities_RearMotor:
        return getMotor(entity).getProperty(property);
    }
    return -1;
}

bool getEntityPropertyChanged(Entities entity, Properties property)
{
    switch (entity)
    {
    case Entities_LeftMotor:
    case Entities_RightMotor:
    case Entities_RearMotor:
        return getMotor(entity).getPropertyChanged(property);
    default:
        return false;
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS)
        Serial.println("Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *pData, int len)
{
    Packet packet;
    int cnt = len / sizeof(Packet);
    while (cnt > 0)
    {
        memcpy(&packet, pData, sizeof(Packet));
        pData += sizeof(Packet);
        cnt--;
        setEntityProperty(packet.entity, packet.property, packet.value);
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

    memcpy(peerInfo.peer_addr, ctrlMacAddress, sizeof(peerInfo.peer_addr));
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

        for (Entities e = Entities_LeftMotor; e <= Entities_RearMotor; e++)
            Motors[e-Entities_LeftMotor].Loop(dmsec);
    }
    dmsec = msec - timeStatusLast;
    if (dmsec >= 500)
    {
        timeStatusLast = msec;

        const int len = 10;
        Packet packets[len];     // enough for a few properties
        uint8_t cnt = 0;
        for (Entities entity = Entities_LeftMotor; entity <= Entities_RearMotor; entity++)
        {
            for (Properties prop = Properties_Goal; prop <= Properties_DirectDrive; prop++)
            {
                if (getEntityPropertyChanged(entity, prop))
                {
                    if (cnt >= len)
                    {
                        floge("packet buffer too small");
                        break;
                    }
                    if (prop != Properties_Goal && prop != Properties_DirectDrive)   // skip writeonly
                    {
                        packets[cnt].entity = entity;
                        packets[cnt].property = prop;
                        packets[cnt].value = getEntityProperty(entity, prop);
                        cnt++;
                    }
                }
            }
        }
        if (cnt > 0)
        {
            esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t*)&packets, cnt * sizeof(Packet));
            if (result != ESP_OK)
                floge("Error sending data");
        }
    }
}
