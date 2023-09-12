#pragma once

//здесь прямая и обратная кинематика для разных конечностей

#include <inttypes.h>

namespace kinematics
{
    enum CalculationResult : ::uint8_t
    {
        CALC_SUCCESSFULL=0,
        CALC_ERROR,
    };

    CalculationResult legIK(/*some args*/);
    CalculationResult legFK(/*some args*/);

    CalculationResult handIK(/*some args*/);
    CalculationResult handFK(/*some args*/);

}