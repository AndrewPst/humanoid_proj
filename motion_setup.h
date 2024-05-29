#pragma once

#include <Arduino.h>
#include "src/kinematics/kinematics.h"

#pragma region Default position

#define DEFAULT_LEG_Z 150
#define DEFAULT_LEG_X 4
#define DEFAULT_LEG_LEDGE_Y 15
#define DEFAULT_LEG_CONVERGANCE 8 * DEG_TO_RAD
#define DEFAULT_LEG_COLLAPSE 0
#define DEFAULT_LEG_CASTER -2 * DEG_TO_RAD

#define DEFAULT_HAND_ANGLE 40 * DEG_TO_RAD
#define DEFAULT_HAND_R 145
#define DEFAULT_HAND_Z 60

#pragma endregion

#pragma region Forward moving parameters

#define FORWARD_MOVING_STEP_FORWARD_SIZE 40
#define FORWARD_MOVING_STEP_BACKWARD_SIZE 20
#define FORWARD_MOVING_STEP_HEIGHT_START 30
#define FORWARD_MOVING_STEP_HEIGHT_END 35
#define FORWARD_MOVING_ONE_LEG_Y_INNER (LEGS_DISTANCE_Y / 4.0)
#define FORWARD_MOVING_ONE_LEG_Y_OUTER (LEGS_DISTANCE_Y / 4.0)
#define FORWARD_MOVING_ONE_LEG_B (0 * DEG_TO_RAD)

#pragma endregion

#pragma region head config

#define HEAD_Y_TO_ANGLE_RATIO

#pragma endregion

#pragma region Arrays

//--------------Default and startup body position-----------
const kinematics::pos_dec_t _default_l_l_pos{
    DEFAULT_LEG_X,
    -DEFAULT_LEG_LEDGE_Y,
    DEFAULT_LEG_Z,
    -DEFAULT_LEG_CONVERGANCE,
    DEFAULT_LEG_COLLAPSE,
    DEFAULT_LEG_CASTER,
};
const kinematics::pos_dec_t _default_l_r_pos{
    DEFAULT_LEG_X,
    DEFAULT_LEG_LEDGE_Y,
    DEFAULT_LEG_Z,
    DEFAULT_LEG_CONVERGANCE,
    DEFAULT_LEG_COLLAPSE,
    DEFAULT_LEG_CASTER,
};
const kinematics::pos_cylindrical_t _default_h_l_pos{
    DEFAULT_HAND_ANGLE,
    DEFAULT_HAND_R,
    DEFAULT_HAND_Z,
};
const kinematics::pos_cylindrical_t _default_h_r_pos{
    DEFAULT_HAND_ANGLE,
    DEFAULT_HAND_R,
    DEFAULT_HAND_Z,
};

#pragma endregion
