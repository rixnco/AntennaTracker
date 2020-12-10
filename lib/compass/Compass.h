#ifndef __COMPASS_H__
#define __COMPASS_H__


#include "Arduino.h"
#include "Wire.h"


class Compass {
  public:
    virtual ~Compass();

	virtual void init() = 0;
    virtual void setMode(byte mode, byte odr, byte rng, byte osr) =0;
    virtual void setReset() =0;
    virtual void read() = 0;

    virtual void setADDR(byte b)  = 0;
	virtual void setSmoothing(byte steps, bool adv)  = 0;
	virtual void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max)  = 0;

	virtual int getX() = 0;
	virtual int getY() = 0;
	virtual int getZ() = 0;
	virtual int getAzimuth() = 0;
	virtual byte getBearing(int azimuth) = 0;
	virtual void getDirection(char* myArray, int azimuth) = 0;
protected:
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


class BaseCompass : public Compass {
public:
    virtual ~BaseCompass();

	virtual void init() override;

    virtual void setADDR(byte b) override;
	virtual void setSmoothing(byte steps, bool adv) override;
	virtual void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max) override;

	virtual int getX() override;
	virtual int getY() override;
	virtual int getZ() override;
	virtual int getAzimuth() override;
	virtual byte getBearing(int azimuth) override;
	virtual void getDirection(char* myArray, int azimuth) override;


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
    
};

class CompassProxy : public Compass {
public:
    CompassProxy(Compass* compass=nullptr) : _compass(compass) {};

    void setCompass(Compass* compass) { _compass= compass; }

	virtual void init() { if(_compass!=nullptr) _compass->init(); };
    virtual void setMode(byte mode, byte odr, byte rng, byte osr) { if(_compass!=nullptr) _compass->setMode(mode,odr,rng,osr); };
    virtual void setReset() { if(_compass!=nullptr) _compass->setReset(); };
    virtual void read()  { if(_compass!=nullptr) _compass->read(); };

    virtual void setADDR(byte b) { if(_compass!=nullptr) _compass->setADDR(b); };
	virtual void setSmoothing(byte steps, bool adv) { if(_compass!=nullptr) _compass->setSmoothing(steps, adv); };
	virtual void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max)  { if(_compass!=nullptr) _compass->setCalibration(x_min,x_max,y_min,y_max,z_min,z_max); };

	virtual int getX() { if(_compass!=nullptr) return _compass->getX(); return 0; };
	virtual int getY() { if(_compass!=nullptr) return _compass->getY(); return 0; };
	virtual int getZ() { if(_compass!=nullptr) return _compass->getZ(); return 0; };
	virtual int getAzimuth() { if(_compass!=nullptr) return _compass->getAzimuth(); return 0; };;
	virtual byte getBearing(int azimuth) { if(_compass!=nullptr) return _compass->getBearing(azimuth); return 0; };
	virtual void getDirection(char* myArray, int azimuth) { if(_compass!=nullptr) _compass->getDirection(myArray, azimuth); };
protected:
    Compass* _compass;
};


#endif // __COMPASS_H__