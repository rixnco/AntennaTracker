#ifndef __COMPASS_H__
#define __COMPASS_H__


#include "Arduino.h"
#include "Wire.h"


class Compass {
	
  public:
    virtual ~Compass();

	virtual void init();
    virtual void setMode(byte mode, byte odr, byte rng, byte osr) =0;
    virtual void setReset() =0;
    virtual void read() = 0;

    void setADDR(byte b);
	void setSmoothing(byte steps, bool adv);
	void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);

	int getX();
	int getY();
	int getZ();
	int getAzimuth();
	byte getBearing(int azimuth);
	void getDirection(char* myArray, int azimuth);


  protected:
    void _writeReg(byte reg,byte val);
	int _get(int index);
	bool _smoothUse = false;
	byte _smoothSteps = 5;
	bool _smoothAdvanced = false;
    byte _ADDR;
	int _vRaw[3] = {0,0,0};
	int _vHistory[10][3];
	int _vScan = 0;
	long _vTotals[3] = {0,0,0};
	int _vSmooth[3] = {0,0,0};
	void _smoothing();
	bool _calibrationUse = false;
	int _vCalibration[3][2];
	int _vCalibrated[3];
	void _applyCalibration();
	const char _bearings[16][3] =  {
		{' ', ' ', 'N'},
		{'N', 'N', 'E'},
		{' ', 'N', 'E'},
		{'E', 'N', 'E'},
		{' ', ' ', 'E'},
		{'E', 'S', 'E'},
		{' ', 'S', 'E'},
		{'S', 'S', 'E'},
		{' ', ' ', 'S'},
		{'S', 'S', 'W'},
		{' ', 'S', 'W'},
		{'W', 'S', 'W'},
		{' ', ' ', 'W'},
		{'W', 'N', 'W'},
		{' ', 'N', 'W'},
		{'N', 'N', 'W'},
	};
	
	
	
};


#endif // __COMPASS_H__