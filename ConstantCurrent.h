#pragma once
#include <Arduino.h>
#include "Cubic1.7.h"
#include "MovingAverage.h"

class Constant_current_controller
{
private:
    //! Number of values used for smoothing. The default value shall be 10.
    const unsigned int smoothingReadings;

    //! Proportional gain of the controller
    const double Kp;

    //! Current value (in analogRead units) that is 0A.
    const unsigned int neutralCurrent;

    //! The duty capable by the motor in use. It should be smaller than or equal to 255 when using with Arduino Mega.
    const unsigned int capableDuty;

    const uint8_t motorPin = A0;

    //! Pointer to the Cubic_motor object
    Cubic_motor *motor;

    //! Target current value (in analogRead units) of the motor
    unsigned int targetCurrent;

    unsigned int current;

    Moving_average<unsigned int> currentMovingAverage;

    //! Current duty value of the motor
    int duty;

public:
    /**
     * @brief Construct a new Constant_current_controller object
     *
     * @param motor  Pointer to the Cubic_motor object
     * @param targetCurrent Target current value (in analogRead units) of the motor
     * @param capableDuty The duty capable by the motor in use. It should be smaller than or equal to 255 when using with Arduino Mega.
     * @param Kp Proportional gain of the controller
     * @param neutralCurrent Current value (in analogRead units) that is 0A. The default value is 511.
     * @param smoothingReadings Number of values used for smoothing. The default value is 10.
     */
    Constant_current_controller(Cubic_motor *motor, unsigned int targetCurrent, unsigned int capableDuty, double Kp, unsigned int neutralCurrent = 511, unsigned int smoothingReadings = 10);
    /**
     * @brief Construct a new Constant_current_controller object
     *
     * @param motor  Pointer to the Cubic_motor object
     * @param capableDuty The duty capable by the motor in use. It should be smaller than or equal to 255 when using with Arduino Mega.
     * @param Kp Proportional gain of the controller
     * @param neutralCurrent Current value (in analogRead units) that is 0A. The default value is 511.
     * @param smoothingReadings Number of values used for smoothing. The default value is 10.
     */
    Constant_current_controller(Cubic_motor *motor, unsigned int capableDuty, double Kp, unsigned int neutralCurrent = 511, unsigned int smoothingReadings = 10);

    /**
     * @brief Set the target current value (in analogRead units) of the motor
     *
     * @param targetCurrent
     */
    void setTargetCurrent(int targetCurrent);
    /**
     * @brief Get the Target Current object
     *
     * @return int targetCurrent
     */
    int getTargetCurrent() const { return this->targetCurrent; }

    /**
     * @brief Update the controller
     * @details This function should be called in the loop() function.
     */
    void update();

    void plot() const;

    /**
     * @brief This function is used to limit the value in the range of [min, max].
     *
     * @tparam T
     * @tparam U
     * @param value
     * @param min
     * @param max
     * @return T
     */
    template <typename T, typename U>
    static T limitInRange(const T value, const U min, const U max);
};

inline void Constant_current_controller::setTargetCurrent(const int targetCurrent)
{
    this->targetCurrent = targetCurrent;
}