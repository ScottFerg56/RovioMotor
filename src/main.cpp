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

uint8_t ctrlMacAddress[] = {0xE8, 0x9F, 0x6D, 0x22, 0x02, 0xEC};

esp_now_peer_info_t peerInfo;

DriveMotor& getMotor(int8_t entity)
{
    return Motors[entity - Entities_LeftMotor];
}

void setEntityProperty(int8_t entity, int8_t property, int8_t value)
{
    switch (entity)
    {
    case Entities_LeftMotor:
    case Entities_RightMotor:
    case Entities_RearMotor:
        getMotor(entity).setProperty(property, value);
        break;

    case Entities_AllMotors:
        for (int8_t m = Entities_LeftMotor; m <= Entities_RearMotor; m++)
            getMotor(m).setProperty(property, value);
        break;
    
    default:    // invalid enity
        break;
    }
}

int8_t getEntityProperty(int8_t entity, int8_t property)
{
    switch (entity)
    {
    case Entities_LeftMotor:
    case Entities_RightMotor:
    case Entities_RearMotor:
        return getMotor(entity).getProperty(property);

    case Entities_AllMotors:    // invalid
    default:    // invalid enity
        return -1;
    }
}

bool getEntityPropertyChanged(int8_t entity, int8_t property)
{
    switch (entity)
    {
    case Entities_LeftMotor:
    case Entities_RightMotor:
    case Entities_RearMotor:
        return getMotor(entity).getPropertyChanged(property);

    case Entities_AllMotors:    // invalid
    default:    // invalid enity
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
    if (len == 0 || (len % 4) != 0)
    {
        Serial.println("invalid data packet length");
        return;
    }
    while (len >= 4)
    {
        uint8_t entity = *pData++;
        uint8_t property = *pData++;
        int16_t value = *pData++;
        value |= *pData++ << 8;
        len -= 4;
        setEntityProperty(entity, property, value);
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

        for (int8_t e = Entities_LeftMotor; e <= Entities_RearMotor; e++)
            Motors[e-Entities_LeftMotor].Loop(dmsec);
    }
    dmsec = msec - timeStatusLast;
    if (dmsec >= 500)
    {
        timeStatusLast = msec;
        uint8_t packet[32];     // enough for at least 4 entities, with 2 properties and 4 bytes each
        uint8_t *p = packet;
        uint8_t len = 0;
        for (int8_t entity = Entities_LeftMotor; entity <= Entities_RearMotor; entity++)
        {
            for (int8_t prop = MotorProperties_Goal; prop <= MotorProperties_DirectDrive; prop++)
            {
                if (getEntityPropertyChanged(entity, prop))
                {
                    int16_t value = getEntityProperty(entity, prop);
                    if (prop != MotorProperties_RPM && prop != MotorProperties_Power)   // skip readonly
                    {
                        if (len + 4 >= sizeof(packet))
                        {
                            Serial.println("packet buffer too small");
                            break;
                        }
                        else
                        {
                            *p++ = entity;
                            *p++ = prop;
                            *p++ = (uint8_t)(value & 0xFF);
                            *p++ = (uint8_t)(value >> 8);
                            len += 4;
                        }
                    }
                }
            }
        }
        if (len > 0)
        {
            esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t*)&packet, len);
            if (result != ESP_OK)
                Serial.println("Error sending the data");
        }
    }
}
