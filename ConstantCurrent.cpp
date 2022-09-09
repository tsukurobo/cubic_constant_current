#pragma once
#include "ConstantCurrent.h"
#include "MovingAverage.h"

void Constant_current_controller::update()
{
    static int deviation = 0;

    current=analogRead(motorPin);

    currentMovingAverage.add(current);

    deviation = targetCurrent - currentMovingAverage.get();
    duty += Kp * deviation;

    duty=limitInRange(duty,(int)capableDuty);
    motor->put(duty);
    motor->send();
}

void Constant_current_controller::plot() const
{
    Serial.print("targetCurrent: ");
    Serial.print(targetCurrent);
    Serial.print(", Moving Current: ");
    Serial.print(currentMovingAverage.get());
    Serial.print(", Duty: ");
    Serial.print(duty);
    Serial.print(", Current: ");
    Serial.println(current);
}

Constant_current_controller::Constant_current_controller(Cubic_motor *motor, unsigned int targetCurrent, unsigned int capableDuty, double Kp, unsigned int neutralCurrent, unsigned int smoothingReadings) : motor(motor), targetCurrent(targetCurrent), neutralCurrent(neutralCurrent), capableDuty(capableDuty), Kp(Kp), smoothingReadings(smoothingReadings)
{
    currentMovingAverage.initialize(analogRead(A0), smoothingReadings);
}
Constant_current_controller::Constant_current_controller(Cubic_motor *motor, unsigned int capableDuty, double Kp, unsigned int neutralCurrent, unsigned int smoothingReadings) : Constant_current_controller(motor, analogRead(A0), capableDuty, Kp, neutralCurrent, smoothingReadings)
{
}
