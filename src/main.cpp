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
Adafruit_DCMotor* HeadMotor;

int16_t HeadGoal = 0;
int16_t HeadPower = 0;
bool HeadPowerChanged = false;

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
    case Entities_Head:
        switch (property)
        {
        case Properties_Goal:
            HeadGoal = value;
            break;
        }
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
    case Entities_Head:
        switch (property)
        {
        case Properties_Power:
            return HeadPower;
        }
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
    case Entities_Head:
        switch (property)
        {
        case Properties_Power:
            {
                bool changed = HeadPowerChanged;
                HeadPowerChanged = false;
                return changed;
            }
        }
        break;
    }
    return false;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS)
        floge("Delivery Fail");
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
    FLogger::setLogLevel(FLOG_DEBUG);
    timeMotorLast = millis();
    Serial.begin(115200);
    delay(1000);    // for Serial

    flogi("In setup");

    // Right Motor has encoder on opposite side from other motors, so encoder pins must be reversed WRT motor polarity
    getMotor(Entities_LeftMotor ).Init(4, 14, A4);
    getMotor(Entities_RightMotor).Init(3, A5, 32);
    getMotor(Entities_RearMotor ).Init(1, A2, A3);

    HeadMotor = DriveMotor::MotorShield.getMotor(2);

    flogi("WIFI init");
    if (!WiFi.mode(WIFI_STA))
        flogf("%s FAILED", "WIFI init");

    flogi("MAC addr: %s", WiFi.macAddress().c_str());

    flogi("ESP_NOW init");
    if (esp_now_init() != ESP_OK)
        flogf("%s FAILED", "ESP_NOW init");

    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo.peer_addr, ctrlMacAddress, sizeof(peerInfo.peer_addr));
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
        flogf("%s FAILED", "ESP_NOW peer add");
    
    esp_now_register_recv_cb(OnDataRecv);
    flogi("completed");
    delay(1000);
}

void loop()
{
    unsigned long msec = millis();
    unsigned long dmsec = msec - timeMotorLast;
    if (dmsec >= 100)
    {
        timeMotorLast = msec;

        for (Entities e = Entities_LeftMotor; e <= Entities_RearMotor; e++)
            getMotor(e).Loop(dmsec);
        if (HeadPower != HeadGoal)
        {
            HeadPowerChanged = true;
            HeadPower = HeadGoal;
            HeadMotor->setSpeed(abs(HeadPower));
            HeadMotor->run(HeadPower == 0 ? RELEASE : (HeadPower < 0 ? BACKWARD : FORWARD));
        }
        u_int16_t val = analogRead(37);
        Serial.printf(">Head Power:%i\r\n", HeadPower);
        Serial.printf(">Head Pos:%i\r\n", val);
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
