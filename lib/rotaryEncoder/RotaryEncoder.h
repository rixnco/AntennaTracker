//
// Created by Samuel HEIDMANN on 27/02/2021.
//

#ifndef ANTENNATRACKER_ROTARYENCODER_H
#define ANTENNATRACKER_ROTARYENCODER_H

#include "Arduino.h"

class RotaryEncoder {
    public:
        virtual ~RotaryEncoder();
        virtual void init() = 0;
        virtual void read() = 0;
        virtual float getAngleDegrees() = 0;
        static float getOffsetedAngle(float angle);
        void setOffest(float offset); // Offest is defined as target value - actual value

    protected:
        float _angle_offset = 0;
};

#endif //ANTENNATRACKER_ROTARYENCODER_H
