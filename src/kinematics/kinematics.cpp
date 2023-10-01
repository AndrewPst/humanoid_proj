#include "kinematics.h"
#include "../../construction.h"
#include "math.h"
#include <Arduino.h>
#include "../../sout.h"

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

namespace kinematics
{
    CalculationResult legIK(const struct pos_t &pos, struct leg_t *out, IKCalcConfig config)
    {
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

        if (out)
        {
            // save out
            out->values[0] = pos.a;
            out->values[1] = y_koef_rad;
            out->values[2] = s_1;
            out->values[3] = s_3;
            out->values[4] = -s_2 - pos.g;
            out->values[5] = y_koef_rad + pos.b;
        }

        return CalculationResult::CALC_SUCCESSFULL;
    }

    CalculationResult legFK(/*some args*/)
    {
        return CalculationResult::CALC_SUCCESSFULL;
    }

}