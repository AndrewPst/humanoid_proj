#pragma once

#define PROG_ATTACKING 0
#define PROG_GOALKEEPER 1

#define PROG_DEBUG_LEG_IK 2
#define PROG_DEBUG_HAND_IK 3
#define PROG_DEBUG_CUM_PARSE 4
#define PROG_DEBUG_ORIENT 5

#define PROG_MOD 0

#define DEBUG_LOG 0
#define DEBUG_BOUD 115200

#define AX_12A 1

#define DYNAMIXEL_ROTATION_TO_UNIT 10.0
#define DEGREES_PER_SEC_2_RPM (1.0 / 6.0)

#define SERVO_TYPE AX_12A

#define CUM_DATA_SIZE_BYTES 16
#define CUM_DATA_OFFSET_BYTES 16
#define CUM_FPS 28
#define CUM_UPDATE_PERIOD_MS (1000 / CUM_FPS)

#define CUM_USE_KALMAN 1
#define CAM_KALMAN_ERR_MEASURE 5
#define CAM_KALMAN_Q 0.1
#define CAM_USE_KALMAN_FOR_CX 1
#define CAM_USE_KALMAN_FOR_CY 1
#define CAM_USE_KALMAN_FOR_AREA 1
#define CUM_OBJECTS_COUNT 1

#define CUM_TRACKING_ACC_COEF 1.15
#define CUM_X_RESOULTION 320
#define CUM_Y_RESOULTION 240
#define CUM_X_VIEWING_ANGLE 90
#define CUM_Y_VIEWING_ANGLE 60
#define CUM_X_DEAD_ZONE 30
#define CUM_Y_DEAD_ZONE 30

#define ORIENT_UPDATE_PERIOD 20

// #define ORIENT_ACC_TRESHOLD_VALUE 4000

#define ORIENT_USE_KALMAN 1
#define ORIENT_KALMAN_ERR_MEASURE 3.0
#define ORIENT_KALMAN_Q 1.0

#define ORIENT_INCIDENCE_AXIS _orient_data_absolute.x
#define ORIENT_COMPASS_AXIS _orient_data_absolute.z

#define FALLING_BACK_TRESHOLD -100
#define FALLING_BACK_DIR -1

#define FALLING_FRONT_TRESHOLD -60
#define FALLING_FRONT_DIR 1

#define FALLING_ANGLE_OFFSET -90

#define USE_RTOS 1 // если надо будет подключать openCM и у нее хватит памяти, то ставим 0, но этого скорее всего не будет