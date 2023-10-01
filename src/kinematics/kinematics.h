#pragma once

// здесь прямая и обратная кинематика для разных конечностей

#include <inttypes.h>
#include <array>
#include "../../construction.h"

namespace kinematics
{
    typedef enum CalculationResult : ::uint8_t
    {
        CALC_SUCCESSFULL = 0,
        CALC_UNREACHABLE_COORDS,
        CALC_ANGLE_RANGE_EXCEEDED,
    };

    typedef enum IKCalcConfig : ::uint8_t
    {
        IKCONF_CHECK_UNREACHABLE_COORDS = 0b0000001,
        IKCONF_CHECK_ANGLE_RANGE_EXCEED = 0b0000010,
        IKCONF_ALL = 0xff
    };

    template <std::size_t _d_c>
    struct limb_t
    {
        std::array<double, _d_c> values;
    };

    struct leg_t : public limb_t<DRIVERS_IN_LEG_COUNT>
    {
    };

    struct pos_t
    {
        double x{0}, y{0}, z{0};
        double a{0}, b{0}, g{0};
        pos_t(double _x = 0, double _y = 0, double _z = 0, double _a = 0, double _b = 0, double _g = 0) : x(_x), y(_y), z(_z), a(_a), b(_b), g(_g) {}
    };

    CalculationResult legIK(const struct pos_t &, struct leg_t *, IKCalcConfig config = IKCalcConfig::IKCONF_ALL);
    CalculationResult legFK(/*some args*/);
    // {
    //     Serial.println("in");
    //     return CalculationResult::CALC_SUCCESSFULL;
    // }

    CalculationResult handIK(/*some args*/);
    CalculationResult handFK(/*some args*/);

}