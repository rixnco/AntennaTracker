#ifndef QMC5883L_Compass
#define QMC5883L_Compass

#include "Compass.h"

#define QMC5883_ADDR                (0x0D)

//  MODE CONTROL (MODE)
#define QMC5883_MODE_SINGLE         (0x00)
#define QMC5883_MODE_CONTINUOUS     (0x01)

// OUTPUT DATA RATE (ODR)
#define QMC5883_ODR_10_HZ           (0x00)
#define QMC5883_ODR_50_HZ           (0x04)
#define QMC5883_ODR_100_HZ          (0x08)
#define QMC5883_ODR_200_HZ          (0x0C)

// FULL SCALE (RNG)
#define QMC5883_RNG_2_GA            (0x00)
#define QMC5883_RNG_8_GA            (0x10)

// OVER SAMPLE RATIO (OSR)
#define QMC5883_OSR_512             (0x00)
#define QMC5883_OSR_256             (0x40)
#define QMC5883_OSR_128             (0x80)
#define QMC5883_OSR_64              (0xC0)



class QMC5883LCompass : public BaseCompass {
	
  public:
    QMC5883LCompass();
    virtual void init();
    virtual void setMode(byte mode, byte odr, byte rng, byte osr);
    virtual void setReset();
    virtual void read();
	
};


#endif
