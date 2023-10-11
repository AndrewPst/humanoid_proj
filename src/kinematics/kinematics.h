#pragma once

#include <inttypes.h>
#include <array>
#include "../../construction.h"

#if LEG_DRIVERS_COUNT != 6
#error Leg inverse kinematics not support this count of drivers
#endif

#if HAND_DRIVERS_COUNT != 3
#error Hand inverse kinematics not support this count of drivers
#endif

/// @brief calculation of kinematics and dynamix of robot movement
namespace kinematics
{
    typedef enum CalculationResult : ::uint8_t
    {
        CALC_SUCCESSFULL = 0,
        CALC_UNREACHABLE_COORDS,
        CALC_ANGLE_RANGE_EXCEEDED,
    };

    /// @brief IK calculations flags
    typedef enum IKCalcConfig : ::uint8_t
    {
        IKCONF_CHECK_UNREACHABLE_COORDS = 0b0000001,
        IKCONF_CHECK_ANGLE_RANGE_EXCEED = 0b0000010,
        IKCONFIG_MIRROR_OUT = 0b00000100,
        IKCONFIG_USE_LEFT_HAND_COOR_SYSTEM = 0b00001000,
        IKCONF_DEFAULT = IKCONF_CHECK_UNREACHABLE_COORDS | IKCONF_CHECK_ANGLE_RANGE_EXCEED,
    };

    using leg_t = std::array<double, LEG_DRIVERS_COUNT>;
    using hand_t = std::array<double, HAND_DRIVERS_COUNT>;

    /// @brief cartesian coordinate system
    struct pos_t
    {
        double x{0}, y{0}, z{0};
        double a{0}, b{0}, g{0};
        pos_t(double _x = 0, double _y = 0, double _z = 0, double _a = 0, double _b = 0, double _g = 0) : x(_x), y(_y), z(_z), a(_a), b(_b), g(_g) {}
    };

    /// @brief cylindrical coordinate system
    struct pos_cylindrical_t
    {
        double a{0}, r{0}, z{0};
        pos_cylindrical_t(double _a = 0, double _r = 0, double _z = 0) : a(_a), r(_r), z(_r) {}
    };

    /// @brief set the pointer to min values of servo in leg (in rad) buffer
    /// @param buf pointer to buffer
    void set_min_values_legs_joints(const double *buf);

    /// @brief set the pointer to max values of servo in leg (in rad) buffer
    /// @param buf pointer to buffer
    void set_max_values_legs_joints(const double *buf);

    /// @brief set the pointer to min values of servo in hand (in rad) buffer
    /// @param buf pointer to buffer
    void set_min_values_hands_joints(const double *buf);

    /// @brief set the pointer to max values of servo in hand (in rad) buffer
    /// @param buf pointer to buffer
    void set_max_values_hands_joints(const double *buf);

    /// @brief calculate inverse kinematics for 6DOF leg
    /// @param pos pos of effector
    /// @param out angles of joints in rad
    /// @param config calculation config
    /// @return result of calculation ik
    CalculationResult legIK(const struct pos_t &pos, leg_t *out, IKCalcConfig config = IKCalcConfig::IKCONF_DEFAULT);

    /// @brief EMPTY function. Do not use. TODO: make realization
    /// @param in -
    /// @param out -
    /// @param config -
    /// @return -
    CalculationResult legFK(const leg_t &in, struct pos_t *out, IKCalcConfig config = IKCalcConfig::IKCONF_DEFAULT);

    /// @brief calculate inverse kinematics for 3DOF hand
    /// @param pos pos of effector
    /// @param out angles of joints in rad
    /// @param config calculation config
    /// @return result of calculation ik
    CalculationResult handIK(const struct pos_cylindrical_t &pos, hand_t *out, IKCalcConfig config = IKCalcConfig::IKCONF_DEFAULT);

    /// @brief EMPTY function. Do not use. TODO: make realization
    /// @param in -
    /// @param out -
    /// @param config -
    /// @return -
    CalculationResult handFK(/*some args*/);

    /// @brief Calculate motion time in sec by moving speed
    /// @param lastpos last position of effector
    /// @param newpos new position of effector
    /// @param speed speed in mm/s OR deg/s (if only rotation)
    /// @return time in sec
    double get_motion_time_by_speed(const struct pos_t &lastpos, const struct pos_t &newpos, const double speed);

    double get_motion_time_by_speed(const struct pos_cylindrical_t &lastpos, const struct pos_cylindrical_t &newpos, const double speed);

    /// @brief Calculate joint rotation speed by time
    /// @param last_pos last position in rad
    /// @param new_pos new position in rad
    /// @param time time of rotation
    /// @return speed of joints rotation in deg/s
    double calc_joint_velocity_by_time(const double last_pos, const double new_pos, const double time);

    /// @brief Calculate joint rotation speed in dynamixel units by time
    /// @param lastpos last position in rad
    /// @param new_pos new position in rad
    /// @param out out buffer in dynamixel units
    /// @param count count of data
    /// @param time time of rotation
    /// @return count of calculated data
    uint8_t calc_joint_velocity_by_time_arr(const double *lastpos, const double *new_pos, int32_t *out, uint8_t count, const double time);

}