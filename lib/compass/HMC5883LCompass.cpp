/*
===============================================================================================================
HMC5883LCompass.h
Library for using HMC5583L series chip boards as a compass.


Supports:

- Getting values of XYZ axis.
- Calculating Azimuth.
- Getting 16 point Azimuth bearing direction (0 - 15).
- Getting 16 point Azimuth bearing Names (N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW)
- Smoothing of XYZ readings via rolling averaging and min / max removal.
- Optional chipset modes (see below)

===============================================================================================================

Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]

*/

#include "HMC5883LCompass.h"


HMC5883LCompass::HMC5883LCompass() {
    setADDR(HMC5883_ADDR);
}


/**
	INIT
	Initialize Chip - This needs to be called in the sketch setup() function.
	
	@since v0.1;
**/
void HMC5883LCompass::init(){
	Wire.begin();
    setReset();
}


/**
	CHIP MODE
	Set the chip mode.
	
	@since v0.1;
**/
// Set chip mode
void HMC5883LCompass::setMode(byte mode, byte odr, byte rng, byte osr){
	_writeReg(0x00, osr | odr); 
    _writeReg(0x01, rng); 
    _writeReg(0x02, mode); 
    delay(10);	
}


/**
	RESET
	Reset the chip.
	
	@since v0.1;
**/
// Reset the chip
void HMC5883LCompass::setReset(){
    // N/A
    // Default config
    setMode(HMC5883_MODE_SINGLE, HMC5883_ODR_15_HZ, HMC5883_RNG_1_30_GA, HMC5883_OSR_1);
}

/**
	READ
	Read the XYZ axis and save the values in an array.
	
	@since v0.1;
**/
void HMC5883LCompass::read(){
	Wire.beginTransmission(_ADDR);
	Wire.write(0x03);
	int err = Wire.endTransmission();
	if (!err) {
		Wire.requestFrom(_ADDR, (byte)6);
        while (Wire.available() < 6);
		_vRaw[0] = (int16_t)((Wire.read() <<8 ) | (Wire.read() & 0x00FF));
		_vRaw[2] = (int16_t)((Wire.read() <<8 ) | (Wire.read() & 0x00FF));
		_vRaw[1] = (int16_t)((Wire.read() <<8 ) | (Wire.read() & 0x00FF));

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

