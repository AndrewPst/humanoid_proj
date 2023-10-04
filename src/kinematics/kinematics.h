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
        IKCONFIG_MIRROR_OUT = 0b00000100,
        IKCONFIG_USE_LEFT_HAND_COOR_SYSTEM = 0b00001000,
        IKCONF_ALL = 0xff
    };

    using leg_t = std::array<double, LEG_DRIVERS_COUNT>;

    struct pos_t
    {
        double x{0}, y{0}, z{0};
        double a{0}, b{0}, g{0};
        pos_t(double _x = 0, double _y = 0, double _z = 0, double _a = 0, double _b = 0, double _g = 0) : x(_x), y(_y), z(_z), a(_a), b(_b), g(_g) {}
    };

    void set_min_values_legs_joints(const double *buf);
    void set_max_values_legs_joints(const double *buf);

    CalculationResult legIK(const struct pos_t &, leg_t *, IKCalcConfig config = IKCalcConfig::IKCONF_ALL);
    CalculationResult legFK(/*some args*/);

    CalculationResult handIK(/*some args*/);
    CalculationResult handFK(/*some args*/);

    double get_motion_time_by_speed(const struct pos_t &lastpos, const struct pos_t &newpos, const double speed);

    double calc_joint_speed_by_time(const double last_pos, const double new_pos, const double time);

}