#include <Arduino.h>
#include "AccelStepper.h"
#include "QMC5883LCompass.h"
#include "GpsUtils.h"
#define MAX_SPEED   3000
#define SPEED       1000
#define STEPS_PER_REVOLUTION 8152
#define CALIB_TARGET STEPS_PER_REVOLUTION + 1000
#define HOME_LAT 45.477577
#define HOME_LONG 6.057257

AccelStepper motor = AccelStepper(AccelStepper::DRIVER, 32, 33);
QMC5883LCompass compass;

//GpsPt HomePt = GpsPt(39.099912, -94.581213, 0);
GpsPt HomePt = GpsPt(HOME_LAT, HOME_LONG, 0);
//GpsPt TargetPt = GpsPt(38.627089, -90.200203, 0);
GpsPt TargetPt = GpsPt(45.472752, 6.097065, 0);
int calibrationData[2][2];

void calibrate(int loops) {
    int x, y;
    calibrationData[0][0] = 999999;
    calibrationData[0][1] = -999999;
    calibrationData[1][0] = 999999;
    calibrationData[1][1] = -999999;
    bool changed;
    // spare first values
    for (int i=0; i<10; i++) {
        compass.read();
        delay(10);
    }
    int loops_done = 0;
    int target = CALIB_TARGET;
    motor.setCurrentPosition(0);
    // Make N loops (back and forth)
    while (loops_done < loops) {
        motor.setSpeed(target == 0 ? -SPEED : SPEED);
        while(motor.currentPosition() != target)
        {
            motor.runSpeed();
            compass.read();
            changed = false;
            // Return XYZ readings
            x = compass.getX();
            y = compass.getY();
            if(x < calibrationData[0][0]) {
                calibrationData[0][0] = x;
                changed = true;
            }
            if(x > calibrationData[0][1]) {
                calibrationData[0][1] = x;
                changed = true;
            }

            if(y < calibrationData[1][0]) {
                calibrationData[1][0] = y;
                changed = true;
            }
            if(y > calibrationData[1][1]) {
                calibrationData[1][1] = y;
                changed = true;
            }

            if (changed) {
                Serial.println("CALIBRATING... Keep moving your sensor around.");
            }
        }
        if(target == 0) {
            target = CALIB_TARGET;
            loops_done += 1;
        }
        else {target = 0;}
    }
    // Now we have finish spinning
    Serial.println("Done");
    Serial.print("compass.setCalibration(");
    Serial.print(calibrationData[0][0]);
    Serial.print(", ");
    Serial.print(calibrationData[0][1]);
    Serial.print(", ");
    Serial.print(calibrationData[1][0]);
    Serial.print(", ");
    Serial.print(calibrationData[1][1]);
    Serial.println(");");
}

float getHeadingError(float current_heading, float target_heading) {
    float errorDeg;
    float rawError = target_heading - current_heading;
    if(rawError < -180.) {
        errorDeg = -rawError - 180;
        return errorDeg;
    } else if(rawError > 180.) {
        errorDeg = -rawError + 180;
        return errorDeg;
    }
    else {
        return rawError;
    }
}

void setup() {

    Serial.begin(115200);
    Serial.println("Starting Antenna Tracker");
    motor.setMaxSpeed(MAX_SPEED);
    compass.init();
    compass.setMode(0x01,0x00,0x00,0xC0);
    motor.stop();
    Serial.println("Started");
    float target_heading = compute_azimuth(HomePt, TargetPt);
    Serial.println("azimuth to target : ");
    Serial.println(target_heading);
    calibrate(1);
    compass.setCalibration(calibrationData[0][0], calibrationData[0][1], calibrationData[1][0], calibrationData[1][1], -2000, 2000);
}

void loop() {
    int a;
    compass.read();

    // Return Azimuth reading
    a = compass.getAzimuth();

    Serial.print("Azimuth: ");
    Serial.print(a);

    float target_heading = compute_azimuth(HomePt, TargetPt);
    float error = getHeadingError(a, target_heading);
    Serial.print("     error: ");
    Serial.print(error);
    Serial.println();
    float speed = error * 500;
    if(speed > MAX_SPEED) {speed = MAX_SPEED;}
    if(speed < -MAX_SPEED) {speed = -MAX_SPEED;}
    motor.setSpeed(speed);
    motor.runSpeed();
//    delay(50);

//    while(motor.currentPosition() != STEPS_PER_REVOLUTION/2 + 1000)
//    {
//        motor.setSpeed(SPEED);
//        motor.runSpeed();
//    }
//    delay(1000);
//    while(motor.currentPosition() != 0 - 1000)
//    {
//        motor.setSpeed(-SPEED);
//        motor.runSpeed();
//    }
//    delay(1000);
}
