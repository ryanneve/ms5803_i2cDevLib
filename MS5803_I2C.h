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

#ifndef _MS5803_H_
#define _MS5803_H_

#define I2CDEV_SERIAL_DEBUG

#include "I2Cdev.h"
#include <avr/pgmspace.h>

#define MS5803_ADDRESS
// Should convert these to enum
#define MS5803_ADDRESS_AD0_LOW     0x77 // address pin low (GND), default for InvenSense evaluation board
#define MS5803_ADDRESS_AD0_HIGH    0x76 // address pin high (VCC)

#define MS5803_DEFAULT_ADDRESS     MS5803_ADDRESS_AD0_LOW

#define MS5803_RESET     0x1E

#define MS5803_PROM_BASE 0xA0
#define MS5803_PROM_C1   0xA2
#define MS5803_PROM_C2   0xA4
#define MS5803_PROM_C3   0xA6
#define MS5803_PROM_C4   0xA8
#define MS5803_PROM_C5   0xAA
#define MS5803_PROM_C6   0xAC
#define MS5803_PROM_CRC  0xAE



#define MS5803_ADC_READ  0x00
#define CMD_ADC_CONV 0x40 // ADC conversion command

// Define measurement type.
enum measurement
{
	PRESSURE    = 0x00,
	TEMPERATURE = 0x10
};

// Define constants for Conversion precision
enum precision
{
	ADC_256  = 0x00,
	ADC_512  = 0x02,
	ADC_1024 = 0x04,
	ADC_2048 = 0x06,
	ADC_4096 = 0x08
};

enum ms5803_model
{
	INVALID =  0,
	BA01    =  1,
	BA02    =  2,
	BA05    =  5,
	BA14    = 14,
	BA30    = 30
};

class MS5803 {
	public:
		MS5803();
		MS5803(uint8_t address);
	
		void		setAddress(uint8_t address);
		uint8_t		getAddress() {return _dev_address;}
		bool		initialize(uint8_t model);
		bool		initialize(uint8_t model, bool debug);
		bool		testConnection();
		uint16_t	reset();
		void		setAtmospheric(float pressure) {press_atmospheric = pressure;}
		void		calcMeasurements(precision _precision) {calcMeasurements(_precision,false);}
		void		calcMeasurements(precision _precision,bool debug);
		// getters
		int32_t		getD1Pressure() { return _d1_pressure; }
		int32_t		getD2Temperature() { return _d2_temperature;}
		float		getTemp_C() {return temp_C;}
		float		getPress_mBar() {return press_mBar;}
		float		getPress_kPa() {return press_kPa;}
		bool		initialized() {return _initialized;}
		
		uint16_t	resolution;
		float		temp_C;
		float		press_mBar;
		float		press_kPa;
		float		press_atmospheric;
		float		press_gauge;
		float		press_psi;
		float		depth_fresh;
	protected:
	private:
		void		_getCalConstants(bool debug);
		int32_t		_getCalConstant(uint8_t constant_no);
		int32_t		_getADCconversion(measurement _measurement, precision _precision, bool debug);
		// Calibration Constants
		int32_t		_c1_SENSt1; // Pressure Sensitivity
		int32_t		_c2_OFFt1;  // Pressure Offset
		int32_t		_c3_TCS;    // Temperature coefficient of pressure sensitivity
		int32_t		_c4_TCO;    // Temperature coefficient of pressure offset
		int32_t		_c5_Tref;   // Reference Temperature
		int32_t		_c6_TEMPSENS;   // Temperature coefficient of the temperature
		// pressure and temperature data
		int32_t		_d1_pressure;    // AdcPressure
		int32_t		_d2_temperature; // AdcTemperature
		// Calculated values
		int32_t		_dT; //TempDifference
		int32_t		_TEMP;       // Actual temperature -40 to 85C with .01 resolution (divide by 100) - Temperature float
		// Temperature compensated pressure
		int64_t		_OFF;       // First Order Offset at actual temperature // Offset - float
		int64_t		_SENS;      // Sensitivity at actual temperature // Sensitivity - float
		int32_t		_P;         // Temperature compensated pressure 10...1300 mbar (divide by 100 to get mBar)
		uint8_t		_buffer[14];
		uint8_t		_dev_address;
		ms5803_model	_model; // the suffix after ms5803. E.g 2 for MS5803-02 indicates range.
		bool		_initialized;
};

// Function prototype:
void serialPrintln64(int64_t val64);
void serialPrint64(int64_t val64);


#endif