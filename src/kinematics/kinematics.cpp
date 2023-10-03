#include "kinematics.h"
#include <Arduino.h>
#include "math.h"
#include "../../construction.h"
#include "../../sout.h"

#include <array>

// replace this caluculation to const numbers for optimization
#define N_COXA_HYPO sqrt(COXA_X *COXA_X + COXA_Z * COXA_Z)
#define N_FEMUR_HYPO sqrt(FEMUR_Z *FEMUR_Z + COXA_X * COXA_X)
#define POW_2_N_COXA_HYPO (n_coxa_hypo * n_coxa_hypo)
#define POW_2_N_FEMUR_HYPO (n_femur_hypo * n_femur_hypo)
#define ATAN_COXA_X__COXA_Z atan(COXA_X / COXA_Z)
#define ATAN_COXA_X__FEMUR_Z atan(COXA_X / FEMUR_Z)
#define ATAN_COXA_Z__COXA_X atan(COXA_Z / COXA_X)
#define ATAN_FEMUR_Z__COXA_X atan(FEMUR_Z / COXA_X)

#define ERROR_FLOATING_POINT_CALC 0.01

struct DriverMetadata_t
{
    const double _min;
    const double _max;
    const double _offset;
    DriverMetadata_t(const double min, const double max, const double offset) : _min(min), _max(max), _offset(offset) {}
};

static const DriverMetadata_t driversMetadata[LEG_DRIVERS_COUNT]{
    DriverMetadata_t{LEG_MIN_ANGLE_0_SERVO, LEG_MAX_ANGLE_0_SERVO, LEG_OFFSET_ANGLE_0_SERVO},
    DriverMetadata_t{LEG_MIN_ANGLE_1_SERVO, LEG_MAX_ANGLE_1_SERVO, LEG_OFFSET_ANGLE_1_SERVO},
    DriverMetadata_t{LEG_MIN_ANGLE_2_SERVO, LEG_MAX_ANGLE_2_SERVO, LEG_OFFSET_ANGLE_2_SERVO},
    DriverMetadata_t{LEG_MIN_ANGLE_3_SERVO, LEG_MAX_ANGLE_3_SERVO, LEG_OFFSET_ANGLE_3_SERVO},
    DriverMetadata_t{LEG_MIN_ANGLE_4_SERVO, LEG_MAX_ANGLE_4_SERVO, LEG_OFFSET_ANGLE_4_SERVO},
    DriverMetadata_t{LEG_MIN_ANGLE_5_SERVO, LEG_MAX_ANGLE_5_SERVO, LEG_OFFSET_ANGLE_5_SERVO},
};

namespace kinematics
{
    CalculationResult legIK(const struct pos_t &pos, leg_t *out, IKCalcConfig config)
    {
        std::array<double, LEG_DRIVERS_COUNT> calcbuf;
        // constans definition
        const double n_z_a = pos.z - TIBIA * cos(pos.b) * cos(pos.g);                //(new z angle) - Angle-abjusted Z position
        const double n_z_hypo = sqrt(n_z_a * n_z_a + pos.x * pos.x + pos.y * pos.y); // calculate hypotenuse lenght for 2,3,4 ids drivers
        const double n_coxa_hypo{N_COXA_HYPO}, n_femur_hypo{N_FEMUR_HYPO};

        // pos calculation (read rotations matrix wiki)
        const double pos_x = pos.x * cos(pos.a) - pos.y * sin(pos.a) - TIBIA * sin(pos.g);
        const double pos_y = pos.x * sin(pos.a) + pos.y * cos(pos.a) + TIBIA * sin(pos.b);

        // out calculation
        double x_koef_rad = 0;
        if (abs(pos_x) > ERROR_FLOATING_POINT_CALC)
            x_koef_rad = M_PI_2 - atan2(n_z_a, pos_x);
        double y_koef_rad = 0;
        if (abs(pos_y) > ERROR_FLOATING_POINT_CALC)
            y_koef_rad = M_PI_2 - atan2(n_z_a, pos_y);

        const double n_z_hypo_pow_2 = n_z_hypo * n_z_hypo;
        double s_1 = acos((POW_2_N_COXA_HYPO + n_z_hypo_pow_2 - POW_2_N_FEMUR_HYPO) / (2.0 * n_coxa_hypo * n_z_hypo)) + ATAN_COXA_X__COXA_Z + x_koef_rad;
        double s_2 = acos((POW_2_N_FEMUR_HYPO + n_z_hypo_pow_2 - POW_2_N_COXA_HYPO) / (2.0 * n_femur_hypo * n_z_hypo)) + ATAN_COXA_X__FEMUR_Z - x_koef_rad;
        double s_3 = 2.0 * M_PI - acos((POW_2_N_FEMUR_HYPO + POW_2_N_COXA_HYPO - n_z_hypo_pow_2) / (2.0 * n_femur_hypo * n_coxa_hypo)) - ATAN_FEMUR_Z__COXA_X - ATAN_COXA_Z__COXA_X;

        calcbuf[0] = pos.a;
        calcbuf[1] = y_koef_rad;
        calcbuf[2] = s_1;
        calcbuf[3] = s_3;
        calcbuf[4] = -s_2 - pos.g;
        calcbuf[5] = y_koef_rad + pos.b;

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

        if (config & IKCalcConfig::IKCONF_CHECK_ANGLE_RANGE_EXCEED)
        {
            for (uint8_t i = 0; i < calcbuf.size(); i++)
            {
                SERIAL_BEGIN
                SERIAL_OUT("ID: ")
                SERIAL_OUT(i)
                SERIAL_OUT(" Calc: ")
                SERIAL_OUT(calcbuf[i])
                SERIAL_OUT(" min: ")
                SERIAL_OUT(driversMetadata[i]._min)
                SERIAL_OUT(" max: ")
                SERIAL_OUT_L(driversMetadata[i]._max)
                SERIAL_END
                if (calcbuf[i] < driversMetadata[i]._min || calcbuf[i] > driversMetadata[i]._max)
                {
                    return CalculationResult::CALC_ANGLE_RANGE_EXCEEDED;
                }
            }
        }

        return CalculationResult::CALC_SUCCESSFULL;
    }

    CalculationResult legFK(/*some args*/)
    {
        return CalculationResult::CALC_SUCCESSFULL;
    }

}