#pragma once

#include <Arduino.h>

#pragma region Default position

#define DEFAULT_LEG_Z 140
#define DEFAULT_LEG_X -10
#define DEFAULT_LEG_LEDGE_Y 15
#define DEFAULT_LEG_CONVERGANCE 10 * DEG_TO_RAD
#define DEFAULT_LEG_COLLAPSE 0
#define DEFAULT_LEG_CASTER 0

#define DEFAULT_HAND_ANGLE 0 * DEG_TO_RAD
#define DEFAULT_HAND_R 140
#define DEFAULT_HAND_Z 110

#pragma endregion

#pragma region Forward moving parameters

#define FORWARD_MOVING_STEP_FORWARD_SIZE 40
#define FORWARD_MOVING_STEP_BACKWARD_SIZE 20
#define FORWARD_MOVING_STEP_HEIGHT_START 30
#define FORWARD_MOVING_STEP_HEIGHT_END 35
#define FORWARD_MOVING_ONE_LEG_Y_INNER (LEGS_DISTANCE_Y / 4.0)
#define FORWARD_MOVING_ONE_LEG_Y_OUTER (LEGS_DISTANCE_Y / 4.0)
#define FORWARD_MOVING_ONE_LEG_B (0 * DEG_TO_RAD)

#define FORWARD_MOVING_OPER_0 400
#define FORWARD_MOVING_OPER_1 400
#define FORWARD_MOVING_OPER_2 400
#define FORWARD_MOVING_OPER_3 400
#define FORWARD_MOVING_OPER_4 400
#define FORWARD_MOVING_OPER_5 400
#define FORWARD_MOVING_OPER_6 400

#pragma endregion
