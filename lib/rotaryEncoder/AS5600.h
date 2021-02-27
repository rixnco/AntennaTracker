//
// Created by Samuel HEIDMANN on 27/02/2021.
//

#ifndef ANTENNATRACKER_AS5600_H
#define ANTENNATRACKER_AS5600_H

#include "RotaryEncoder.h"
#include "Wire.h"

#define AS5600_ADDR 0x36
#define AS5600_ZMCO 0x00
#define AS5600_ZPOS_HI 0x01
#define AS5600_ZPOS_LO 0x02
#define AS5600_MPOS_HI 0x03
#define AS5600_MPOS_LO 0x04
#define AS5600_MANG_HI 0x05
#define AS5600_MANG_LO 0x06
#define AS5600_CONF_HI 0x07
#define AS5600_CONF_LO 0x08
#define AS5600_RAW_ANG_HI 0x0C
#define AS5600_RAW_ANG_LO 0x0D
#define AS5600_ANG_HI 0x0E
#define AS5600_ANG_LO 0x0F
#define AS5600_STAT 0x0B
#define AS5600_AGC 0x1A
#define AS5600_MAG_HI 0x1B
#define AS5600_MAG_LO 0x1C
#define AS5600_BURN 0xFF


class AS5600 : public RotaryEncoder {
    public:
        AS5600();
        void init() override;
        void read() override;
        uint16_t getRawAngle() override;
        float getAngleDegrees() override;
        byte getStatus() const;

    protected:
        static uint16_t _read_word(byte addr, byte reg);
        static uint8_t  _read_byte(byte addr, byte reg);
        byte _address;
        uint16_t _reg1;
        byte _reg2;
        byte _reg3;
};


#endif //ANTENNATRACKER_AS5600_H
