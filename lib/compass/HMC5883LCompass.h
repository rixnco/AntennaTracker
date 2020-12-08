#ifndef HMC5883L_Compass
#define HMC5883L_Compass

#include "Compass.h"

#define HMC5883_ADDR                (0x1E)


#define HMC5883_MODE_CONTINUOUS     (0x00)
#define HMC5883_MODE_SINGLE         (0x01)
#define HMC5883_MODE_IDLE           (0x02)

#define HMC5883_ODR_0_75_HZ         (0x00)
#define HMC5883_ODR_1_50_HZ         (0x04)
#define HMC5883_ODR_3_HZ            (0x08)
#define HMC5883_ODR_7_5_H           (0x0C)
#define HMC5883_ODR_15_HZ           (0x10)
#define HMC5883_ODR_30_HZ           (0x14)
#define HMC5883_ODR_75_HZ           (0x18)

#define HMC5883_RNG_0_88_GA         (0x00)
#define HMC5883_RNG_1_30_GA         (0x20)
#define HMC5883_RNG_1_90_GA         (0x40)
#define HMC5883_RNG_2_50_GA         (0x60)
#define HMC5883_RNG_4_00_GA         (0x80)
#define HMC5883_RNG_4_70_GA         (0xA0)
#define HMC5883_RNG_5_60_GA         (0xC0)
#define HMC5883_RNG_8_10_GA         (0xE0)

#define HMC5883_OSR_1               (0x00)
#define HMC5883_OSR_2               (0x20)
#define HMC5883_OSR_4               (0x40)
#define HMC5883_OSR_8               (0x60)



class HMC5883LCompass : public Compass {
	
  public:
    HMC5883LCompass();
    virtual void init();
    virtual void setMode(byte mode, byte odr, byte rng, byte osr);
    virtual void setReset();
    virtual void read();
	
};

#endif
