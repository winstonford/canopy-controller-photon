//
//   SDL_Arduino_INA3221 Library
//   SDL_Arduino_INA3221.cpp Arduino code - runs in continuous mode
//   Version 1.1
//   SwitchDoc Labs   January 31, 2015
//
//



/*
    Initial code from INA219 code (Basically just a core structure left)
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (see BSDlicense.txt)
*/
#include "Particle.h"



#include "lib1.h"

/**************************************************************************/
/*!
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void SDL_Arduino_INA3221::wireWriteRegister (uint8_t board, uint8_t reg, uint16_t value)
{
  // board is the board number, 0 thru 3 if several INA3221 boards are on the same I2C bus.
  Wire.beginTransmission(INA3221_i2caddr + board);
    //Serial.print("Write address = ");
    //Serial.println(INA3221_i2caddr + board);


    Wire.write(reg);                       // Register
    Wire.write((value >> 8) & 0xFF);       // Upper 8-bits
    Wire.write(value & 0xFF);              // Lower 8-bits

  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void SDL_Arduino_INA3221::wireReadRegister(uint8_t board, uint8_t reg, uint16_t *value)
{

  //Serial.print("Read address = ");
  //Serial.println(INA3221_i2caddr + board);

  Wire.beginTransmission(INA3221_i2caddr + board);

    Wire.write(reg);                       // Register
    Wire.endTransmission();

  delay(1); // Max 12-bit conversion time is 586us per sample

  Wire.requestFrom(INA3221_i2caddr + board, (uint8_t)2);

    *value = ((Wire.read() << 8) | Wire.read());

}

//
void SDL_Arduino_INA3221::INA3221SetConfig(uint8_t board)
{


  // Set Config register to take into account the settings above
  uint16_t config = INA3221_CONFIG_ENABLE_CHAN1 |
                    INA3221_CONFIG_ENABLE_CHAN2 |
                    INA3221_CONFIG_ENABLE_CHAN3 |
                    INA3221_CONFIG_AVG1 |
                    INA3221_CONFIG_VBUS_CT2 |
                    INA3221_CONFIG_VSH_CT2 |
                    INA3221_CONFIG_MODE_2 |
                    INA3221_CONFIG_MODE_1 |
                    INA3221_CONFIG_MODE_0;
  wireWriteRegister(board, INA3221_REG_CONFIG, config);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new SDL_Arduino_INA3221 class
*/
/**************************************************************************/
SDL_Arduino_INA3221::SDL_Arduino_INA3221(uint8_t addr, float shuntresistor) {

    INA3221_i2caddr = addr;
    INA3221_shuntresistor = shuntresistor;

}

/**************************************************************************/
/*!
    @brief  Setups the HW (defaults to 32V and 2A for calibration values)
*/
/**************************************************************************/
void SDL_Arduino_INA3221::begin(uint8_t board) {
  Wire.begin();
  // Set chip to known config values to start
  INA3221SetConfig(board);

    Serial.print("shut resistor="); Serial.println(INA3221_shuntresistor);
    Serial.print("address="); Serial.println(INA3221_i2caddr + board);

}

/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t SDL_Arduino_INA3221::getBusVoltage_raw(uint8_t board, int channel) {
  uint16_t value;
  wireReadRegister(board, INA3221_REG_BUSVOLTAGE_1+(channel -1) *2, &value);

//    Serial.print("BusVoltage_raw=");
//    Serial.println(value,HEX);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)(value );
}

/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t SDL_Arduino_INA3221::getShuntVoltage_raw(uint8_t board, int channel) {
  uint16_t value;
  wireReadRegister(board, INA3221_REG_SHUNTVOLTAGE_1+(channel -1) *2, &value);
   // Serial.print("ShuntVoltage_raw=");
   // Serial.println(value,HEX);
  return (int16_t)value;
}



/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in mV (so +-168.3mV)
*/
/**************************************************************************/
float SDL_Arduino_INA3221::getShuntVoltage_mV(uint8_t board, int channel) {
  int16_t value;
  value = getShuntVoltage_raw(board, channel);
  return value * 0.005;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/
float SDL_Arduino_INA3221::getBusVoltage_V(uint8_t board, int channel) {
  int16_t value = getBusVoltage_raw(board, channel);
  return value * 0.001;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
float SDL_Arduino_INA3221::getCurrent_mA(uint8_t board, int channel) {
    float valueDec = getShuntVoltage_mV(board, channel)/INA3221_shuntresistor;

  return valueDec;
}
