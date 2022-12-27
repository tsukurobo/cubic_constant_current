#pragma once
#include <Arduino.h>
#include "cubic.ver1.8.h"
#include "ConstantCurrent.h"

Cubic_motor motor;

void setup()
{
    motor.begin(0);

    Serial.begin(115200);
}

void loop()
{
    // コンストラクタの引数は、順に、モーターのポインタ、初期目標電流、最大出力duty、比例ゲイン、電流センサーの中立電流、平均化する電流の数
    // これ以外の引数を取るコンストラクタも用意されているので、必要に応じてそちらを使ってもよい。
    static Constant_current_controller motorController(&motor, 565, 150, 0.02, 509, 20);

    /* ここで、モーターの出力を更新する。引数にtrueを与えると、putまで行われる。
    falseを与えるか、引数を与えないと、putは行われない。
    引数によらず、現在与えるべきdutyが返される。 */
    motorController.update(true);
    /* 上のコードは、下と同じ意味を持つ。
    int duty = motorController.update();
    motor.put(duty); */

    // モーターの出力や電流などをシリアルモニターに表示する。主にデバッグ用。
    motorController.plot();

    /* sendは自分でする必要がある。 */
    Cubic_motor::send();

    delay(10);
}