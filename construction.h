#pragma once

#include <math.h>
#include <Arduino.h>

static const double LEG_FEMUR_Z = 74;
static const double LEG_FEMUR_X = 14;
static const double LEG_TIBIA_Z = 74;
static const double LEG_TARSUS_Z = 32;

static const double HAND_COXA_Z = 22;
static const double HAND_COXA_R = 13;
static const double HAND_FEMUR_Z = 72;
static const double HAND_TIBIA_Z = 115;

static const double LEGS_DISTANCE_Y = 80;

#define LEG_DRIVERS_COUNT 6
#define HAND_DRIVERS_COUNT 3
#define HEAD_DRIVERS_COUNT 2

#define SERVO_COUNT (2 * LEG_DRIVERS_COUNT + 2 * HAND_DRIVERS_COUNT + HEAD_DRIVERS_COUNT)
//-----------leg-----------

#define LEG_MIN_ANGLE_0_SERVO -M_PI / 3.0
#define LEG_MAX_ANGLE_0_SERVO M_PI / 3.0
#define LEG_OFFSET_ANGLE_0_SERVO M_PI_4

#define LEG_MIN_ANGLE_1_SERVO -M_PI_4
#define LEG_MAX_ANGLE_1_SERVO M_PI_4
#define LEG_OFFSET_ANGLE_1_SERVO 0

#define LEG_MIN_ANGLE_2_SERVO -20 * DEG_TO_RAD
#define LEG_MAX_ANGLE_2_SERVO 3 * M_PI_4
#define LEG_OFFSET_ANGLE_2_SERVO 5 * DEG_TO_RAD

#define LEG_MIN_ANGLE_3_SERVO 0
#define LEG_MAX_ANGLE_3_SERVO 3 * M_PI_4
#define LEG_OFFSET_ANGLE_3_SERVO 0

#define LEG_MIN_ANGLE_4_SERVO -M_PI_2
#define LEG_MAX_ANGLE_4_SERVO M_PI / 6.0
#define LEG_OFFSET_ANGLE_4_SERVO 0

#define LEG_MIN_ANGLE_5_SERVO -50 * DEG_TO_RAD
#define LEG_MAX_ANGLE_5_SERVO 50 * DEG_TO_RAD
#define LEG_OFFSET_ANGLE_5_SERVO 0

//---------hand-----------

#define HAND_MIN_ANGLE_0_SERVO -5 * M_PI / 6.0
#define HAND_MAX_ANGLE_0_SERVO 5 * M_PI / 6.0
#define HAND_OFFSET_ANGLE_0_SERVO 0

#define HAND_MIN_ANGLE_1_SERVO -95 * DEG_TO_RAD
#define HAND_MAX_ANGLE_1_SERVO 100 * DEG_TO_RAD
#define HAND_OFFSET_ANGLE_1_SERVO 0

#define HAND_MIN_ANGLE_2_SERVO -95 * DEG_TO_RAD
#define HAND_MAX_ANGLE_2_SERVO 95 * DEG_TO_RAD
#define HAND_OFFSET_ANGLE_2_SERVO 0

//-----------hand-----------

#define HEAD_MIN_ANGLE_0_SERVO -5 * M_PI / 6.0
#define HEAD_MAX_ANGLE_0_SERVO 5 * M_PI / 6.0
#define HEAD_OFFSET_ANGLE_0_SERVO 0

#define HEAD_MIN_ANGLE_1_SERVO -M_PI_2
#define HEAD_MAX_ANGLE_1_SERVO M_PI_2
#define HEAD_OFFSET_ANGLE_1_SERVO 0

//----------------Backlash compensation

#define LEG_LEFT_BACKLASH_COMP_0_SERVO 1.0
#define LEG_LEFT_BACKLASH_COMP_1_SERVO 1.0
#define LEG_LEFT_BACKLASH_COMP_2_SERVO 1.05
#define LEG_LEFT_BACKLASH_COMP_3_SERVO 0.985
#define LEG_LEFT_BACKLASH_COMP_4_SERVO 1.01
#define LEG_LEFT_BACKLASH_COMP_5_SERVO 1.0

#define LEG_RIGHT_BACKLASH_COMP_0_SERVO 1.0
#define LEG_RIGHT_BACKLASH_COMP_1_SERVO 1.0
#define LEG_RIGHT_BACKLASH_COMP_2_SERVO 1.05
#define LEG_RIGHT_BACKLASH_COMP_3_SERVO 0.985
#define LEG_RIGHT_BACKLASH_COMP_4_SERVO 1.01
#define LEG_RIGHT_BACKLASH_COMP_5_SERVO 1.0

#define HAND_LEFT_BACKLASH_COMP_0_SERVO 1.0
#define HAND_LEFT_BACKLASH_COMP_1_SERVO 1.0
#define HAND_LEFT_BACKLASH_COMP_2_SERVO 1.0

#define HAND_RIGHT_BACKLASH_COMP_0_SERVO 1.0
#define HAND_RIGHT_BACKLASH_COMP_1_SERVO 1.0
#define HAND_RIGHT_BACKLASH_COMP_2_SERVO 1.0

#define HEAD_BACKLASH_COMP_0_SERVO 1.0
#define HEAD_BACKLASH_COMP_1_SERVO 1.0

//----------------Indexes of servos-----------

#define LEG_LEFT_0_DRIVER_ID 8
#define LEG_LEFT_1_DRIVER_ID 10
#define LEG_LEFT_2_DRIVER_ID 12
#define LEG_LEFT_3_DRIVER_ID 14
#define LEG_LEFT_4_DRIVER_ID 16
#define LEG_LEFT_5_DRIVER_ID 18

#define LEG_RIGHT_0_DRIVER_ID 7
#define LEG_RIGHT_1_DRIVER_ID 9
#define LEG_RIGHT_2_DRIVER_ID 11
#define LEG_RIGHT_3_DRIVER_ID 13
#define LEG_RIGHT_4_DRIVER_ID 15
#define LEG_RIGHT_5_DRIVER_ID 17

// #define HAND_LEFT_0_DRIVER_ID 1
// #define HAND_LEFT_1_DRIVER_ID 3
// #define HAND_LEFT_2_DRIVER_ID 5

// #define HAND_RIGHT_0_DRIVER_ID 2 // 2, 4, 6
// #define HAND_RIGHT_1_DRIVER_ID 4
// #define HAND_RIGHT_2_DRIVER_ID 6

#define HAND_LEFT_0_DRIVER_ID 2
#define HAND_LEFT_1_DRIVER_ID 4
#define HAND_LEFT_2_DRIVER_ID 6

#define HAND_RIGHT_0_DRIVER_ID 1 // 2, 4, 6
#define HAND_RIGHT_1_DRIVER_ID 3
#define HAND_RIGHT_2_DRIVER_ID 5

#define HEAD_0_DRIVER_ID 19
#define HEAD_1_DRIVER_ID 20

#define CAM_ID 51
