#pragma once
#include <Arduino.h>
#include "cubic.ver1.8.h"
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

    //! Motor Pin NO.
    const uint8_t motorPin = A0;

    //! Pointer to the Cubic_motor object
    Cubic_motor *motor;

    //! Target current value (in analogRead units) of the motor
    unsigned int targetCurrent;

    //! Latest current value (in analogRead units) of the motor
    unsigned int current;

    //! Moving average of the current value
    Moving_average<unsigned int> currentMovingAverage;

    //! Current duty value of the motor
    double duty;

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
     * @brief Get the Target Current value
     *
     * @return int targetCurrent
     */
    int getTargetCurrent() const { return this->targetCurrent; }

    /**
     * @brief Update the controller
     * @param ifPut If set to true, the duty will be put to the Cubic_motor object. The default value is false.
     * @return int duty
     * @details This function should be called in the loop() function.
     */
    int update(bool ifPut = false);

    /**
     * @brief Plot the current values and duty to the Serial Monitor
     * @details This function does not necessarily need to be called in the loop() function.
     */
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
    static T limitInRange(T value, U min, U max);

    /**
     * @brief This function is used to limit the value in the range of [-|max_absolute|, |max_absolute|].
     *
     * @tparam T
     * @tparam U
     * @param value
     * @param max_absolute
     * @return T
     */
    template <typename T, typename U>
    static T limitInRange(T value, U max_absolute);
};

inline void Constant_current_controller::setTargetCurrent(const int targetCurrent)
{
    this->targetCurrent = targetCurrent;
}

template <typename T, typename U>
inline T Constant_current_controller::limitInRange(const T value, const U min, const U max)
{
    return min(max(value, min), max);
}

template <typename T, typename U>
inline T Constant_current_controller::limitInRange(const T value, const U max_absolute)
{
    if (max_absolute >= 0)
        return limitInRange(value, -1 * max_absolute, max_absolute);
    else
        return limitInRange(value, max_absolute, -1 * max_absolute);
}
