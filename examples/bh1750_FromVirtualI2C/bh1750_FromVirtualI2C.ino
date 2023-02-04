#include "flprog_BH1750.h"

FLProgI2C wireDevice(0);
FLProgTCA9548A commutator(&wireDevice);
FLProgVirtualI2C virtualWire(&commutator, 2);
FLProgBH1750 sensor(&virtualWire);
uint32_t startTime;
uint32_t startTime1;
uint32_t maxCicleTime = 0;
uint32_t startCicleTime = 0;
uint32_t cicleTime = 0;

void setup()
{
    Serial.begin(9600);
    wireDevice.begin();
    startTime = millis() + 3000;
    startTime1 = millis() + 3000;
}

void loop()
{
    if (flprog::isTimer(startTime, 1000))
    {
        Serial.print(F("Light level.........:"));
        Serial.print(sensor.getLightLevel());
        Serial.println(F(" lux"));
        Serial.print("ErrorCode - ");
        Serial.println(sensor.getError());
        Serial.print("maxCicleTime - ");
        Serial.println(maxCicleTime);
        Serial.println();
        startTime = millis();
        sensor.read();
    }
    else
    {
        if (flprog::isTimer(startTime1, 2000))
        {
            startCicleTime = micros();
            sensor.pool();
            cicleTime = micros() - startCicleTime;
            maxCicleTime = max(maxCicleTime, cicleTime);
        }
        else
        {
            sensor.pool();
        }
    }
}
