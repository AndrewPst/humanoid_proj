#include "kinematics.h"
#include <Arduino.h>
#include "math.h"
#include "../../construction.h"
#include "../../sout.h"

#include <array>

// replace this caluculation to const numbers for optimization
#define L_N_COXA_HYPO sqrt(LEG_FEMUR_X *LEG_FEMUR_X + LEG_FEMUR_Z * LEG_FEMUR_Z)
#define L_N_FEMUR_HYPO sqrt(LEG_TIBIA_Z *LEG_TIBIA_Z + LEG_FEMUR_X * LEG_FEMUR_X)
#define L_POW_2_N_COXA_HYPO (n_coxa_hypo * n_coxa_hypo)
#define L_POW_2_N_FEMUR_HYPO (n_femur_hypo * n_femur_hypo)
#define L_ATAN_COXA_X__COXA_Z atan(LEG_FEMUR_X / LEG_FEMUR_Z)
#define L_ATAN_COXA_X__FEMUR_Z atan(LEG_FEMUR_X / LEG_TIBIA_Z)
#define L_ATAN_COXA_Z__COXA_X atan(LEG_FEMUR_Z / LEG_FEMUR_X)
#define L_ATAN_FEMUR_Z__COXA_X atan(LEG_TIBIA_Z / LEG_FEMUR_X)

#define ERROR_FLOATING_POINT_CALC 0.01

namespace kinematics
{
    const double *min_values_legs_joints{nullptr};
    const double *max_values_legs_joints{nullptr};

    const double *min_values_hands_joints{nullptr};
    const double *max_values_hands_joints{nullptr};

    void set_min_values_legs_joints(const double *buf)
    {
        if (!buf)
        {
            SERIAL_OUT_L_THRSAFE("[Kinematics]-Min legs drives values empty!");
        }
        else
            min_values_legs_joints = buf;
    }

    void set_max_values_legs_joints(const double *buf)
    {
        if (!buf)
        {
            SERIAL_OUT_L_THRSAFE("[Kinematics]-Max legs drives values empty!");
        }
        else
            max_values_legs_joints = buf;
    }

    void set_min_values_hands_joints(const double *buf)
    {
        if (!buf)
        {
            SERIAL_OUT_L_THRSAFE("[Kinematics]-Min legs drives values empty!");
        }
        else
            min_values_hands_joints = buf;
    }

    void set_max_values_hands_joints(const double *buf)
    {
        if (!buf)
        {
            SERIAL_OUT_L_THRSAFE("[Kinematics]-Max legs drives values empty!");
        }
        else
            max_values_hands_joints = buf;
    }

    CalculationResult legIK(const struct pos_dec_t &pos, leg_t *out, IKCalcConfig config)
    {
        leg_t calcbuf;

        double koef{1};
        if (config & IKCalcConfig::IKCONFIG_MIRROR_OUT)
            koef = -1;
        // constans definition
        const double n_z_a = pos.z - LEG_TARSUS_Z * cos(pos.b) * cos(pos.g);         //(new z angle) - Angle-abjusted Z position
        const double n_z_hypo = sqrt(n_z_a * n_z_a + pos.x * pos.x + pos.y * pos.y); // calculate hypotenuse lenght for 2,3,4 ids drivers
        const double n_coxa_hypo{L_N_COXA_HYPO}, n_femur_hypo{L_N_FEMUR_HYPO};

        // pos calculation (read rotations matrix wiki)
        const double pos_x = pos.x * cos(pos.a) + pos.y * sin(pos.a * koef) * koef - LEG_TARSUS_Z * sin(pos.g);
        const double pos_y = pos.x * sin(pos.a * koef) - pos.y * cos(pos.a) * koef - LEG_TARSUS_Z * sin(pos.b * koef);

        // out calculation
        double x_koef_rad = 0;
        if (abs(pos_x) > ERROR_FLOATING_POINT_CALC)
            x_koef_rad = M_PI_2 - atan2(n_z_a, pos_x);
        double y_koef_rad = 0;
        if (abs(pos_y) > ERROR_FLOATING_POINT_CALC)
            y_koef_rad = M_PI_2 - atan2(n_z_a, pos_y);

        const double n_z_hypo_pow_2 = n_z_hypo * n_z_hypo;
        double s_1 = acos((L_POW_2_N_COXA_HYPO + n_z_hypo_pow_2 - L_POW_2_N_FEMUR_HYPO) / (2.0 * n_coxa_hypo * n_z_hypo)) + L_ATAN_COXA_X__COXA_Z + x_koef_rad;
        double s_2 = acos((L_POW_2_N_FEMUR_HYPO + n_z_hypo_pow_2 - L_POW_2_N_COXA_HYPO) / (2.0 * n_femur_hypo * n_z_hypo)) + L_ATAN_COXA_X__FEMUR_Z - x_koef_rad;
        double s_3 = 2.0 * M_PI - acos((L_POW_2_N_FEMUR_HYPO + L_POW_2_N_COXA_HYPO - n_z_hypo_pow_2) / (2.0 * n_femur_hypo * n_coxa_hypo)) - L_ATAN_FEMUR_Z__COXA_X - L_ATAN_COXA_Z__COXA_X;

        calcbuf[0] = pos.a;
        calcbuf[1] = koef * y_koef_rad;
        calcbuf[2] = koef * s_1;
        calcbuf[3] = koef * s_3;
        calcbuf[4] = koef * (-s_2 - pos.g);
        calcbuf[5] = koef * (y_koef_rad - pos.b * koef);

        if (out)
        {
            // save out
            *out = calcbuf;
        }

        if (config & IKCalcConfig::IKCONF_CHECK_UNREACHABLE_COORDS)
        {
            for (auto &i : calcbuf)
            {
                if (isnan(i))
                {
                    return CalculationResult::CALC_UNREACHABLE_COORDS;
                }
            }
        }

        if (config & IKCalcConfig::IKCONF_CHECK_ANGLE_RANGE_EXCEED && min_values_legs_joints && max_values_legs_joints)
        {
            for (uint8_t i = 0; i < calcbuf.size(); i++)
            {
                double min{min_values_legs_joints[i]}, max{max_values_legs_joints[i]};
                if (config & IKCalcConfig::IKCONFIG_MIRROR_OUT)
                {
                    std::swap(min, max);
                    min *= -1;
                    max *= -1;
                }
                if (calcbuf[i] < min || calcbuf[i] > max)
                {
                    SERIAL_BEGIN
                    SERIAL_OUT("ID: ")
                    SERIAL_OUT(i)
                    SERIAL_OUT(" Calc: ")
                    SERIAL_OUT(calcbuf[i])
                    SERIAL_OUT(" min: ")
                    SERIAL_OUT(min)
                    SERIAL_OUT(" max: ")
                    SERIAL_OUT_L(max)
                    SERIAL_END
                    return CalculationResult::CALC_ANGLE_RANGE_EXCEEDED;
                }
            }
        }

        return CalculationResult::CALC_SUCCESSFULL;
    }

    CalculationResult legFK(const leg_t &in, struct pos_dec_t *out, IKCalcConfig config)
    {

        return CalculationResult::CALC_SUCCESSFULL;
    }

#define H_FEMUR_Z_POW_2 (HAND_FEMUR_Z * HAND_FEMUR_Z)
#define H_TIBIA_Z_POW_2 (HAND_TIBIA_Z * HAND_TIBIA_Z)

    CalculationResult handIK(const struct pos_cylindrical_t &pos, hand_t *out, IKCalcConfig config)
    {
        hand_t hand;
        double koef = 1, koef_inv = 1;
        if (config & IKCalcConfig::IKCONFIG_MIRROR_OUT)
            koef = -1;
        if (config & IKCalcConfig::IKCONFIG_USE_LEFT_HAND_COOR_SYSTEM)
            koef_inv = -1;

        hand[0] = koef * pos.a;
        double l = sqrt((pos.r - HAND_COXA_R) * (pos.r - HAND_COXA_R) + (pos.z - HAND_COXA_Z) * (pos.z - HAND_COXA_Z));
        hand[1] = koef_inv * -koef * (acos((l * l + H_FEMUR_Z_POW_2 - H_TIBIA_Z_POW_2) / (2.0 * l * HAND_FEMUR_Z)) - asin((pos.r - HAND_COXA_R) / l) * koef_inv);
        hand[2] = koef_inv * koef * (M_PI - acos((H_FEMUR_Z_POW_2 + H_TIBIA_Z_POW_2 - l * l) / (2.0 * HAND_FEMUR_Z * HAND_TIBIA_Z)));

        if (out)
        {
            // save out
            *out = hand;
        }

        if (config & IKCalcConfig::IKCONF_CHECK_UNREACHABLE_COORDS)
        {
            for (auto &i : hand)
            {
                if (isnan(i))
                {
                    return CalculationResult::CALC_UNREACHABLE_COORDS;
                }
            }
        }

        if (config & IKCalcConfig::IKCONF_CHECK_ANGLE_RANGE_EXCEED && min_values_legs_joints && max_values_legs_joints)
        {
            for (uint8_t i = 0; i < hand.size(); i++)
            {
                double min{min_values_hands_joints[i]}, max{max_values_hands_joints[i]};
                if (config & IKCalcConfig::IKCONFIG_MIRROR_OUT)
                {
                    std::swap(min, max);
                    min *= -1;
                    max *= -1;
                }
                if (hand[i] < min || hand[i] > max)
                {
                    SERIAL_BEGIN
                    SERIAL_OUT("ID: ")
                    SERIAL_OUT(i)
                    SERIAL_OUT(" Calc: ")
                    SERIAL_OUT(hand[i])
                    SERIAL_OUT(" min: ")
                    SERIAL_OUT(min)
                    SERIAL_OUT(" max: ")
                    SERIAL_OUT_L(max)
                    SERIAL_END
                    return CalculationResult::CALC_ANGLE_RANGE_EXCEEDED;
                }
            }
        }
    }

    double get_motion_time_by_speed(const struct pos_dec_t &lastpos, const struct pos_dec_t &newpos, const double speed)
    {
        if (speed == 0)
            return 0;
        double time = sqrt((newpos.x - lastpos.x) * (newpos.x - lastpos.x) + (newpos.y - lastpos.y) * (newpos.y - lastpos.y) + (newpos.z - lastpos.z) * (newpos.z - lastpos.z)) / speed;
        if (time == 0)
            time = sqrt((newpos.a - lastpos.a) * (newpos.a - lastpos.a) + (newpos.b - lastpos.b) * (newpos.b - lastpos.b) + (newpos.g - lastpos.g) * (newpos.g - lastpos.g)) * RAD_TO_DEG / speed;
        return time;
    }

    double get_motion_time_by_speed(const struct pos_cylindrical_t &lastpos, const struct pos_cylindrical_t &newpos, const double speed)
    {
        if (speed == 0)
            return 0;
        double time = sqrt((newpos.r - lastpos.r) * (newpos.r - lastpos.r) + (newpos.z - lastpos.z) * (newpos.z - lastpos.z) + ((newpos.a - lastpos.a) * RAD_TO_DEG) * ((newpos.a - lastpos.a) * RAD_TO_DEG)) / speed;
        return time;
    }

    double calc_joint_velocity_by_time(const double last_pos, const double new_pos, const double time)
    {
        return abs(last_pos - new_pos) * RAD_TO_DEG / time;
    }

    uint8_t calc_joint_velocity_by_time_arr(const double *lastpos, const double *new_pos, int32_t *out, uint8_t count, const double time)
    {
        uint8_t i{0};
        for (; i < count; i++)
        {
            out[i] = abs(lastpos[i] - new_pos[i]) * RAD_TO_DEG / time * DEGREES_PER_SEC_2_RPM * DYNAMIXEL_ROTATION_TO_UNIT;
        }
        return i;
    }
}