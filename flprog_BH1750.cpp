#include "flprog_BH1750.h"

FLProgBH1750::FLProgBH1750(FLProgI2C *device, uint8_t i2c_address)
{
    i2cDevice = device;
    addres = i2c_address;
}

void FLProgBH1750::createError()
{
    gotoStepWithDelay(FLPROG_SENSOR_WAITING_READ_STEP, 500);
}

void FLProgBH1750::pool()
{
    checkReadPeriod();
    checkDelay();
    if (step == FLPROG_SENSOR_WAITING_READ_STEP)
    {
        if (isNeededReset)
        {
            resetSensor();
            isNeededReset = false;
        }
        else
        {
            if (currentPower != newPower)
            {
                setPower();
            }
            else
            {
                if (currentSensitivity != newSensitivity)
                {
                    setSensitivity();
                }
                else
                {
                    checkNeededRead();
                }
            }
        }
    }
    if (step == FLPROG_BH1750_READ_SENSOR_STEP1)
    {
        readSensor1();
    }
}

void FLProgBH1750::sensitivity(float sensitivity)
{
    newSensitivity = constrain(sensitivity, FLPROG_BH1750_SENSITIVITY_MIN, FLPROG_BH1750_SENSITIVITY_MAX);
}

bool FLProgBH1750::setSensitivity()
{
    uint8_t valueMTreg = newSensitivity * FLPROG_BH1750_MTREG_DEFAULT; // calculate MTreg value for new sensitivity, measurement time register range 31..254
    uint8_t measurnentTimeHighBit = valueMTreg;
    uint8_t measurnentTimeLowBit = valueMTreg;
    /* high bit manipulation */
    measurnentTimeHighBit >>= 5;                               // 0,0,0,0  0,7-bit,6-bit,5-bit
    measurnentTimeHighBit |= FLPROG_BH1750_MEASUREMENT_TIME_H; // 0,1,0,0  0,7-bit,6-bit,5-bit
                                                               /* low bit manipulation */
    measurnentTimeLowBit <<= 3;                                // 4-bit,3-bit,2-bit,1-bit  0-bit,0,0,0
    measurnentTimeLowBit >>= 3;                                // 0,0,0,4-bit  3-bit,2-bit,1-bit,0-bit
    measurnentTimeLowBit |= FLPROG_BH1750_MEASUREMENT_TIME_L;  // 0,1,1,4-bit  3-bit,2-bit,1-bit,0-bit
    /* update sensor MTreg register */
    codeError = i2cDevice->fullWrite(addres, measurnentTimeHighBit);
    if (codeError)
    {
        createError();
        return false;
    }
    codeError = i2cDevice->fullWrite(addres, measurnentTimeLowBit);
    if (codeError)
    {
        createError();
        return false;
    }
    currentSensitivity = newSensitivity; // MTreg register update success -> update sensitivity value
    return true;
}

void FLProgBH1750::readSensor()
{
    uint32_t delay = 0;
    codeError = i2cDevice->fullWrite(addres, sensorResolution);
    if (codeError)
    {
        createError();
        return;
    }
    switch (sensorResolution)
    {
    case FLPROG_BH1750_ONE_TIME_HIGH_RES_MODE:
    case FLPROG_BH1750_ONE_TIME_HIGH_RES_MODE_2:
        delay = currentSensitivity * 180; // integration time = (0.45..3.68) * 120..180msec -> 81msec/12Hz..662msec/2Hz (default 180msec/5Hz)
        break;
    case FLPROG_BH1750_ONE_TIME_LOW_RES_MODE:
        delay = currentSensitivity * 24; // integration time = (0.45..3.68) * 16..24msec -> 10msec/100Hz..88msec/11Hz (default 24msec/42Hz)
        break;
    }
    gotoStepWithDelay(FLPROG_BH1750_READ_SENSOR_STEP1, delay);
}

void FLProgBH1750::readSensor1()
{
    uint8_t temp[2];
    codeError = i2cDevice->fullRead(addres, temp, 2);
    if (codeError)
    {
        createError();
        return;
    }
    uint16_t rawLightLevel = temp[0] << 8;
    rawLightLevel |= temp[1];
    switch (sensorResolution)
    {
    case FLPROG_BH1750_ONE_TIME_HIGH_RES_MODE_2:

        lightLevel = 0.5 * (float)rawLightLevel / accuracy * currentSensitivity; // 0.50 lux resolution but smaller measurement range
        break;
    case FLPROG_BH1750_ONE_TIME_LOW_RES_MODE:
    case FLPROG_BH1750_ONE_TIME_HIGH_RES_MODE:
        lightLevel = (float)rawLightLevel / accuracy * currentSensitivity; // 1.00 lux & 4.00 lux resolution
        break;
    }
    step = FLPROG_SENSOR_WAITING_READ_STEP;
}

void FLProgBH1750::power(bool power)
{
    newPower = power;
}

void FLProgBH1750::setPower()
{
    if (newPower)
    {
        codeError = i2cDevice->fullWrite(addres, FLPROG_BH1750_POWER_ON);
    }
    else
    {
        codeError = i2cDevice->fullWrite(addres, FLPROG_BH1750_POWER_DOWN);
    }
    if (codeError)
    {
        createError();
        return;
    }
    delayMicroseconds(1);
    currentPower = newPower;
}

void FLProgBH1750::reset()
{
    isNeededReset = true;
}

void FLProgBH1750::resetSensor()
{
    codeError = i2cDevice->fullWrite(addres, FLPROG_BH1750_RESET);
    if (codeError)
    {
        createError();
        return;
    }
    delayMicroseconds(1); // see NOTE
}

/**************************************************************************/
/*
    setCalibration()

    Set sensor calibration value

    NOTE:
    - accuracy range 0.96..1.44

    - typical accuracy values:
      - 1.00, fluorescent light
      - 1.06, white LED & artifical sun
      - 1.15, halogen light
      - 1.18, krypton light light
      - 1.20, incandescent light (by default)

    - accuracy = sensor output lux / actual lux
*/
/**************************************************************************/
void FLProgBH1750::calibration(float newAccuracy)
{
    accuracy = constrain(accuracy, FLPROG_BH1750_ACCURACY_MIN, FLPROG_BH1750_ACCURACY_MAX); // accuracy range 0.96..1.44
}
