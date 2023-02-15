/***************************************************************************************************/
/*
   This is an Arduino library for the ROHM BH1750FVI Ambient Light Sensor

   written by : enjoyneering
   sourse code: https://github.com/enjoyneering/

   ROHM BH1750FVI features:
   - power supply voltage +2.4v..+3.6v, absolute maximum +4.5v
   - maximum current 190uA, sleep current 1uA
   - I2C bus speed 100KHz..400KHz, up to 2 sensors on the bus
   - maximum sensitivity at 560nm, yellow-green light
   - 50Hz/60Hz flicker reduction
   - measurement accuracy +-20%
   - optical filter compensation by changing sensitivity* 0.45..3.68
   - calibration by changing the accuracy 0.96..1.44:
     - typical accuracy values:
       - 1.00, fluorescent light
       - 1.06, white LED & artifical sun
       - 1.15, halogen light
       - 1.18, krypton light light
       - 1.20, incandescent light (by default)
   - onetime+sleep and continuous measurement mode
   - typical measurement resolution:
     - 0.5 lux at high resolution mode2
     - 1.0 lux at high resolution mode (by default)
     - 4.0 lux at low resolution mode
   - typical measurement range depends on resolution mode sensitivity & accuracy values:
     - from 1..32767 to 1..65535 lux
   - typical measurement interval depends on resolution mode & sensitivity:
     - from 81..662 msec to 10..88 msec
     *Any optical filter you put in front of the sensor blocks some light. Sensitivity is used
      to compensate the influence of the optical filter. For example, when transmission rate of
      optical window is 50% (measurement result becomes 0.5 times lower), influence of optical
      window is compensated by changing sensor sensitivity from default 1.0 to 2.0 times

   This device uses I2C bus to communicate, specials pins are required to interface
   Board                                     SDA              SCL              Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4               A5               5v
   Mega2560................................. 20               21               5v
   Due, SAM3X8E............................. 20               21               3.3v
   Leonardo, Micro, ATmega32U4.............. 2                3                5v
   Digistump, Trinket, Gemma, ATtiny85...... PB0/D0           PB2/D2           3.3v/5v
   Blue Pill*, STM32F103xxxx boards*........ PB7/PB9          PB6/PB8          3.3v/5v
   ESP8266 ESP-01**......................... GPIO0            GPIO2            3.3v/5v
   NodeMCU 1.0**, WeMos D1 Mini**........... GPIO4/D2         GPIO5/D1         3.3v/5v
   ESP32***................................. GPIO21/D21       GPIO22/D22       3.3v
                                             GPIO16/D16       GPIO17/D17       3.3v
                                            *hardware I2C Wire mapped to Wire1 in stm32duino
                                             see https://github.com/stm32duino/wiki/wiki/API#I2C
                                           **most boards has 10K..12K pullup-up resistor
                                             on GPIO0/D3, GPIO2/D4/LED & pullup-down on
                                             GPIO15/D8 for flash & boot
                                          ***hardware I2C Wire mapped to TwoWire(0) aka GPIO21/GPIO22 in Arduino ESP32

   Supported frameworks:
   Arduino Core - https://github.com/arduino/Arduino/tree/master/hardware
   ATtiny  Core - https://github.com/SpenceKonde/ATTinyCore
   ESP8266 Core - https://github.com/esp8266/Arduino
   ESP32   Core - https://github.com/espressif/arduino-esp32
   STM32   Core - https://github.com/stm32duino/Arduino_Core_STM32


   GNU GPL license, all text above must be included in any redistribution,
   see link for details - https://www.gnu.org/licenses/licenses.html
*/
/**********************************************************/

#pragma once
#include "Arduino.h"
#include "flprogUtilites.h"

#define FLPROG_BH1750_POWER_DOWN 0x00         // low power state register
#define FLPROG_BH1750_POWER_ON 0x01           // wake-up & wating for measurment command register
#define FLPROG_BH1750_RESET 0x07              // soft reset register
#define FLPROG_BH1750_MEASUREMENT_TIME_H 0x40 // changing measurement time MSB-bits register
#define FLPROG_BH1750_MEASUREMENT_TIME_L 0x60 // changing measurement time LSB-bits register

#define FLPROG_BH1750_SENSITIVITY_MIN 0.45     // minimun sensitivity value
#define FLPROG_BH1750_SENSITIVITY_MAX 3.68     // maximum sensitivity value
#define FLPROG_BH1750_SENSITIVITY_DEFAULT 1.00 // default sensitivity value, used to calculate MTreg value

#define FLPROG_BH1750_MTREG_DEFAULT 0x45 // default integration/measurement time value, 69
#define FLPROG_BH1750_MTREG_MIN 0x1F     // minimun integration/measurement time value, 31
#define FLPROG_BH1750_MTREG_MAX 0xFE     // maximum integration/measurement time value, 254

#define FLPROG_BH1750_ACCURACY_MIN 0.96     // minimun accuracy value
#define FLPROG_BH1750_ACCURACY_MAX 1.44     // maximum accuracy value
#define FLPROG_BH1750_ACCURACY_DEFAULT 1.20 // default measurement accuracy value for incandescent light

/* misc */
#define FLPROG_BH1750FVI_I2C_SPEED_HZ 100000   // sensor I2C speed 100KHz..400KHz, in Hz
#define FLPROG_BH1750FVI_I2C_STRETCH_USEC 1000 // I2C stretch time, in usec
#define FLPROG_BH1750_ERROR 0xFFFFFFFF

#define FLPROG_BH1750_DEFAULT_I2CADDR 0x23 // device I2C address if address pin LOW
#define FLPROG_BH1750_SECOND_I2CADDR 0x5C  // device I2C address if address pin HIGH

// #define FLPROG_BH1750_CONTINUOUS_HIGH_RES_MODE 0x10   // continuous measurement register, 1.0 lx resolution
// #define FLPROG_BH1750_CONTINUOUS_HIGH_RES_MODE_2 0x11 // continuous measurement register, 0.5 lx resolution
// #define FLPROG_BH1750_CONTINUOUS_LOW_RES_MODE 0x13    // continuous measurement register, 4.0 lx resolution

#define FLPROG_BH1750_ONE_TIME_HIGH_RES_MODE 0x20   // one measurement & sleep register, 1.0 lx resolution
#define FLPROG_BH1750_ONE_TIME_HIGH_RES_MODE_2 0x21 // one measurement & sleep register, 0.5 lx resolution
#define FLPROG_BH1750_ONE_TIME_LOW_RES_MODE 0x23    // one measurement & sleep register, 4.0 lx resolution

#define FLPROG_BH1750_WAITING_READ_STEP 0
#define FLPROG_BH1750_WAITING_DELAY 1
#define FLPROG_BH1750_READ_SENSOR_STEP1 2

#define FLPROG_BH1750_NOT_ERROR 0


class FLProgBH1750
{
public:
  FLProgBH1750(FLProgI2C *device, uint8_t i2c_address = FLPROG_BH1750_DEFAULT_I2CADDR);
  void pool();
  float getSensitivity() { return currentSensitivity; };
  float getCalibration() { return accuracy; };
  float getLightLevel() { return lightLevel; };
  void power(bool power);
  void reset();
  void read();
  uint8_t getError() { return codeError; };
  void sensitivity(float sensitivity);
  void calibration(float newAccuracy);
  void setReadPeriod(uint32_t period);

private:
  bool setSensitivity();
  void checkDelay();
  void createError();
  void readSensor();
  void readSensor1();
  void setPower();
  void resetSensor();
  FLProgI2C *i2cDevice;
  uint8_t step = FLPROG_BH1750_WAITING_READ_STEP;
  uint32_t startDelay;
  uint32_t sizeDelay;
  uint8_t stepAfterDelay;
  uint8_t sensorResolution = FLPROG_BH1750_ONE_TIME_HIGH_RES_MODE;
  float currentSensitivity = 0;
  float newSensitivity = FLPROG_BH1750_SENSITIVITY_DEFAULT;
  float accuracy = FLPROG_BH1750_ACCURACY_DEFAULT;
  uint8_t sensorAddress;
  bool isNeededRead = true;
  bool isNeededReset = false;
  bool currentPower = true;
  bool newPower = true;
  uint8_t codeError = FLPROG_BH1750_NOT_ERROR;
  float lightLevel;
  uint32_t readPeriod = 0;
  uint32_t startReadPeriod = 0;
};