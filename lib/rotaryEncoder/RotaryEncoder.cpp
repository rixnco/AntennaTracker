//
// Created by Samuel HEIDMANN on 27/02/2021.
//

#include "RotaryEncoder.h"


RotaryEncoder::~RotaryEncoder() {

}

void RotaryEncoder::setOffest(float offset) {
    _angle_offset = offset;
}

float RotaryEncoder::getOffsetedAngle(float angle) {
    float offsted_angle = fmodf(angle, 360.);
    if(offsted_angle < 0) {
        offsted_angle += 360.;
    }
    return offsted_angle;
}
