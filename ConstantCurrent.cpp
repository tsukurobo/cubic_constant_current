#pragma once
#include "ConstantCurrent.h"
#include "MovingAverage.h"

template <typename T, typename U>
inline T Constant_current_controller::limitInRange(const T value, const U min, const U max)
{
    return min(max(value, min), max);
}

void Constant_current_controller::update()
{
    current = analogRead(motorPin);
    currentMovingAverage.add(current);

    duty = limitInRange(duty + Kp * (double)(targetCurrent - currentMovingAverage.get()), -1 * capableDuty, capableDuty);
    motor->put(duty);
    Cubic_motor::send();
}

void Constant_current_controller::plot() const
{
    Serial.print("targetCurrent: ");
    Serial.print(targetCurrent);
    Serial.print(", Moving Current: ");
    Serial.print(currentMovingAverage.get());
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
