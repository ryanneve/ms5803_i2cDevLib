// I2Cdev library collection - MS5803 I2C device class
// Based on Measurement Specialties MS5803 document, 3/25/2013 (DA5803-01BA_010)
// 4/12/2014 by Ryan Neve <Ryan@PlanktosInstruments.com>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2014 Ryan Neve

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "MS5803_I2C.h"


/** Default constructor, uses default I2C address.
 * @see MPU6050_DEFAULT_ADDRESS
 */
MS5803::MS5803() {
    devAddr = MS5803_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see MS5803_DEFAULT_ADDRESS
 * @see MS5803_ADDRESS_AD0_LOW
 * @see MS5803_ADDRESS_AD0_HIGH
 */
MS5803::MS5803(uint8_t address) {
    devAddr = address;
}

/** Power on and prepare for general usage.
 * This will reset the device to make sure that the calibration PROM gets loaded into 
 * the internal register. It will then read the PROM
 */
void MS5803::initialize(uint8_t model) {
    _model = model;
    reset();
    _getCalConstants();
    _debug = false;
}

bool MS5803::testConnection(){
  bool result;
  result =  I2Cdev::writeBytes(devAddr,MS5803_D2_512,0,buffer);
  return result;
}

void MS5803::reset(){
  // Not sure how to send no buffer to an address.
  bool result = I2Cdev::writeBytes(devAddr, MS5803_RESET,0,buffer);
}

void MS5803::_getCalConstants(){
  I2Cdev::readBytes(devAddr,MS5803_PROM_C1,2,buffer);
  _c1_SENSt1 = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(devAddr,MS5803_PROM_C2,2,buffer);
  _c2_OFFt1 = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(devAddr,MS5803_PROM_C3,2,buffer);
  _c3_TCS = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(devAddr,MS5803_PROM_C4,2,buffer);
  _c4_TCO = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(devAddr,MS5803_PROM_C5,2,buffer);
  _c5_Tref = (((uint16_t)buffer[0] << 8) + buffer[1]);
  I2Cdev::readBytes(devAddr,MS5803_PROM_C6,2,buffer);
  _c6_TEMPSENS = (((uint16_t)buffer[0] << 8) + buffer[1]);
}

void MS5803::debugCalConstants(){
  Serial.print("_c1_SENSt1 "); Serial.println(_c1_SENSt1);
  Serial.print("_c2_OFFt1 "); Serial.println(_c2_OFFt1);
  Serial.print("_c3_TCS "); Serial.println(_c3_TCS);
  Serial.print("_c4_TCO "); Serial.println(_c4_TCO);
  Serial.print("_c5_Tref "); Serial.println(_c5_Tref);
  Serial.print("_c6_TEMPSENS "); Serial.println(_c6_TEMPSENS);
}

int32_t MS5803::getTemperature(bool debug){
  _debug = debug;
  return getTemperature();
}
int32_t MS5803::getTemperature(){
  char buf[14];
  int64_t T2 = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;
  // Request which buffer and oversampling ratio...
  bool result =   I2Cdev::writeBytes(devAddr, MS5803_D2_512,0,buffer);
  // Get buffer & convert
  uint8_t count = I2Cdev::readBytes(devAddr,MS5803_ADC_READ,3,buffer,2000);
  if ( _debug ) { Serial.print("    getTemperature buffer "); Serial.print(buffer[0],HEX); Serial.write(' ' ); Serial.print(buffer[1],HEX); Serial.write(' ' ); Serial.println(buffer[2],HEX);}
  _d2_temperature = ((uint32_t)buffer[0] << 16) + ((uint32_t)buffer[1] << 8) + (uint32_t)buffer[2];
  // now some calculations
  _dT = _d2_temperature - (_c5_Tref * 256);
  int64_t offset1 =  _dT * _c6_TEMPSENS; //
  int64_t offset2 = offset1 / pow(2,23); //(1<<23);
  _TEMP = 2000 + offset2;
  if ( _debug ) {
    Serial.println("First order values:");
    dtostrf(_d2_temperature,13,0,buf);
    Serial.print("    _d2_temperature = "); Serial.println(buf);
    Serial.print("      _c5_Tref = "); Serial.println(_c5_Tref);
    dtostrf(_dT,13,0,buf);
    Serial.print("    _dT = "); Serial.println(buf);
    Serial.print("      _c6_TEMPSENS = "); Serial.println(_c6_TEMPSENS);
    dtostrf(offset1,13,0,buf);
    Serial.print("    offset1 = "); Serial.println(buf);
    dtostrf(offset2,13,0,buf);
    Serial.print("    offset2 = "); Serial.println(buf);
    Serial.print("    _TEMP = "); Serial.println(_TEMP);
  }
  switch (_model) {
    case (1):  //MS5803-01-----------------------------------------------------------
      _OFF  = (int64_t)_c2_OFFt1  * (1<<16) + ((int64_t)_dT * (int64_t)_c4_TCO) / (1<<7);
      _SENS = (int64_t)_c1_SENSt1 * (1<<15) + ((int64_t)_dT * (int64_t)_c3_TCS) / (1<<8);
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = pow(_dT,2)/(1<<31);
        OFF2 = 3 * pow((_TEMP - 2000.0),2);
        SENS2 = 7 * pow((_TEMP - 2000.0),2) /  8;
        if ( _TEMP < 1500.0 ) SENS2 += 2 * pow((_TEMP + 1500.0),2);// below 15C
      }
      else if ( _TEMP > 4500.0 ) SENS2 -= pow((_TEMP - 4500.0),2)  /8;
      break;
    case (2):  //MS5803-02-----------------------------------------------------------
      _OFF  = (int64_t)_c2_OFFt1  * (1<<17) + ((int64_t)_dT * (int64_t)_c4_TCO) / (1<<6);
      _SENS = (int64_t)_c1_SENSt1 * (1<<16) + ((int64_t)_dT * (int64_t)_c3_TCS) / (1<<7);
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = pow(_dT,2)/(1<<31);
        OFF2 = 61 * pow((_TEMP - 2000.0),2)/(1<<4);
        SENS2 = 2 * pow((_TEMP - 2000.0),2);
        if ( _TEMP < 1500.0 ) { // below 15C
          OFF2  += 20 * pow((_TEMP + 1500.0),2);
          SENS2 += 12 * pow((_TEMP + 1500.0),2);
        }
      }
      else if ( _TEMP > 4500.0 ) SENS2 -= pow((_TEMP - 4500.0),2)  /8;
      break;
    case (5):  //MS5803-05-----------------------------------------------------------
      _OFF  = (int64_t)_c2_OFFt1  * (1<<18) + ((int64_t)_dT * (int64_t)_c4_TCO) / (1<<5);
      _SENS = (int64_t)_c1_SENSt1 * (1<<17) + ((int64_t)_dT * (int64_t)_c3_TCS) / (1<<7);
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2)/(1<<33);
        OFF2 = 3 * pow((_TEMP - 2000.0),2) / (1<<3);
        SENS2 = 7 * pow((_TEMP - 2000.0),2) / (1<<3);
        if ( _TEMP < 1500.0 ) SENS2 += 3 * pow((_TEMP + 1500.0),2);// below 15C
      }
      break;
    case (14):  //MS5803-14-----------------------------------------------------------
      _OFF  = (int64_t)_c2_OFFt1  * (1<<16) + ((int64_t)_dT * (int64_t)_c4_TCO) / (1<<7);
      _SENS = (int64_t)_c1_SENSt1 * (1<<15) + ((int64_t)_dT * (int64_t)_c3_TCS) / (1<<8);
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2)/(1<<33);
        OFF2 = 3 * pow((_TEMP - 2000.0),2) / 2;
        SENS2 = 5 * pow((_TEMP - 2000.0),2) / (1<<3);
        if ( _TEMP < 1500.0 ) {// below 15C
          OFF2  += 7 * pow((_TEMP + 1500.0),2);
          SENS2 += 4 * pow((_TEMP + 1500.0),2);
        }
      }
      else {
        T2 = 7 * pow(_dT,2)/(1<<37);
        OFF2 = 1 * pow((_TEMP - 2000.0),2) / (1<<4);
        SENS2 = 0;
      }
      break;
    case (30):  //MS5803-30-----------------------------------------------------------
      _OFF  = (int64_t)_c2_OFFt1  * (1<<16) + ((int64_t)_dT * (int64_t)_c4_TCO) / (1<<7);
      _SENS = (int64_t)_c1_SENSt1 * (1<<15) + ((int64_t)_dT * (int64_t)_c3_TCS) / (1<<8);
      // 2nd Order calculations
      if ( _TEMP < 2000.0) {  // Is temperature below or above 20.00 deg C ?
        T2 = 3 * pow(_dT,2)/(1<<33);
        OFF2 = 3 * pow((_TEMP - 2000.0),2) / 2;
        SENS2 = 5 * pow((_TEMP - 2000.0),2) / (1<<3);
        if ( _TEMP < 1500.0 ) {// below 15C
          OFF2  += 7 * pow((_TEMP + 1500.0),2);
          SENS2 += 4 * pow((_TEMP + 1500.0),2);
        }
      }
      else {
        T2 = 7 * pow(_dT,2)/(1<<37);
        OFF2 = 1 * pow((_TEMP - 2000.0),2) / (1<<4);
        SENS2 = 0;
      }
      break;
    default: break;
  }
  _TEMP -= T2;
  _OFF -= OFF2;
  _SENS -= SENS2;
  temp_C = (float)_TEMP / 100.0f;
  if ( _debug ) {
    Serial.println("Second Order Values:");
    Serial.print("    _TEMP = "); Serial.println(_TEMP);
    dtostrf(T2,13,0,buf);
    Serial.print("    T2 = "); Serial.println(buf);
    dtostrf(OFF2,13,0,buf);
    Serial.print("    OFF2 = "); Serial.println(buf);
    dtostrf(SENS2,13,0,buf);
    Serial.print("    SENS2 = "); Serial.println(buf);
    dtostrf(temp_C,13,0,buf);
    Serial.print("    temp_C = "); Serial.println(buf);
  }
  _debug = false; // Always resets
  return _TEMP;
}

int32_t MS5803::getPressure(){
  // Request which buffer and oversampling ratio...
  bool result =   I2Cdev::writeBytes(devAddr, MS5803_D1_512,0,buffer);
  // Get buffer & convert
  uint8_t count = I2Cdev::readBytes(devAddr,MS5803_ADC_READ,3,buffer,2000);
  _d1_pressure = ((uint32_t)buffer[0] << 16) + ((uint32_t)buffer[1] << 8) + (uint32_t)buffer[2];
  //dtostrf(_d1_pressure,13,0,buf);
  //Serial.print("_d1_pressure = "); Serial.println(buf);
  //dtostrf(_SENS,13,0,buf);
  //Serial.print("_SENS = "); Serial.println(buf);
  //dtostrf(_OFF,13,0,buf);
  //Serial.print("_OFF = "); Serial.println(buf);
  _P = (((int64_t)_d1_pressure * _SENS) / (1<<21) - _OFF) / (1<<15);
  //dtostrf(_P,13,0,buf);
  //Serial.print("_P = "); Serial.println(buf);
  press_mBar = _P/100.0f;
  //dtostrf(press_mBar,13,0,buf);
  //Serial.print("press_mBar = "); Serial.println(buf);
  return _P;
}
void MS5803::debugTemperature(){
  getTemperature(true); // Set debug flag
}
