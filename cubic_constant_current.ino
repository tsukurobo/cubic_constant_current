#pragma once
#include <Arduino.h>
#include "Cubic1.7.h"
#include "ConstantCurrent.h"
#include "MovingAverage.h"

Cubic_motor motor;

void setup()
{
    motor.begin(0);

    Serial.begin(115200);
}

void loop()
{
    static Constant_current_controller motorController(&motor, 565, 150, 0.02, 509, 20);
    motorController.update();
    motorController.plot();

    delay(10);
}