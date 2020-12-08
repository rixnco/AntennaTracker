/*
===============================================================================================================
QMC5883LCompass.h
Library for using QMC5583L series chip boards as a compass.
Learn more at [https://github.com/mprograms/QMC5883LCompass]

Supports:

- Getting values of XYZ axis.
- Calculating Azimuth.
- Getting 16 point Azimuth bearing direction (0 - 15).
- Getting 16 point Azimuth bearing Names (N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW)
- Smoothing of XYZ readings via rolling averaging and min / max removal.
- Optional chipset modes (see below)

===============================================================================================================

v1.0 - June 13, 2019
Written by MRPrograms 
Github: [https://github.com/mprograms/]

Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]

===============================================================================================================



FROM QST QMC5883L Datasheet [https://nettigo.pl/attachments/440]
-----------------------------------------------
 MODE CONTROL (MODE)
	Standby			0x00
	Continuous		0x01

OUTPUT DATA RATE (ODR)
	10Hz        	0x00
	50Hz        	0x04
	100Hz       	0x08
	200Hz       	0x0C

FULL SCALE (RNG)
	2G          	0x00
	8G          	0x10

OVER SAMPLE RATIO (OSR)
	512         	0x00
	256         	0x40
	128         	0x80
	64          	0xC0 
  
*/



#include "Arduino.h"
#include "QMC5883LCompass.h"
#include <Wire.h>

QMC5883LCompass::QMC5883LCompass() {
    setADDR(QMC5883_ADDR);
}


/**
	INIT
	Initialize Chip - This needs to be called in the sketch setup() function.
	
	@since v0.1;
**/
void QMC5883LCompass::init(){
	Wire.begin();
	_writeReg(0x0B,0x01);
	setMode(0x01,0x0C,0x10,0X00);
}


/**
	CHIP MODE
	Set the chip mode.
	
	@since v0.1;
**/
// Set chip mode
void QMC5883LCompass::setMode(byte mode, byte odr, byte rng, byte osr){
	_writeReg(0x09,mode|odr|rng|osr);
}


/**
	RESET
	Reset the chip.
	
	@since v0.1;
**/
// Reset the chip
void QMC5883LCompass::setReset(){
	_writeReg(0x0A,0x80);
}

/**
	READ
	Read the XYZ axis and save the values in an array.
	
	@since v0.1;
**/
void QMC5883LCompass::read(){
	Wire.beginTransmission(_ADDR);
	Wire.write(0x00);
	int err = Wire.endTransmission();
	if (!err) {
		Wire.requestFrom(_ADDR, (byte)6);
		_vRaw[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
		_vRaw[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
		_vRaw[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8);

		if ( _calibrationUse ) {
			_applyCalibration();
		}
		
		if ( _smoothUse ) {
			_smoothing();
		}
		
		//byte overflow = Wire.read() & 0x02;
		//return overflow << 2;
	}
}

