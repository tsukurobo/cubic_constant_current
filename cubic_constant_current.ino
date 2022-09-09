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
    static Constant_current_controller motorController(&motor, 150, 200, 0.03, 509,10);
    motorController.update();
    motorController.plot();

    delay(10);
}