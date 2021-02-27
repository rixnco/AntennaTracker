#ifndef FAKE_Compass
#define FAKE_Compass

#include "Compass.h"


class FakeCompass : public BaseCompass {
	
  public:
    FakeCompass() {};
    virtual void init() {};
    virtual void setMode(byte mode, byte odr, byte rng, byte osr) {};
    virtual void setReset() {};
    virtual void read() {};
	
};


#endif
