#pragma once

#define USE_CAMERA 1 // меняем на 0 если надо отключить модули (код для них не будет компилиться для экономии памяти)
#define USE_GYRO 1
#define USE_ACC 1

#define DEBUG_LOG 1
#define DEBUG_BOUD 115200

#define DEFAULT_BODY_Z_POZ 50   // высота туловища над полом
#define DEFAULT_BODY_X_OFFSET 0 // сдвиг туловища вперед для компенсации неустойчивости
#define DEFAULT_BODY_Y_ANGLE 0  // поворот туловища по оси Y для компенсации неустойчивости

#define AUTO_SURFACE_CALIBRATION 0 // автоматическое вычисление параметров выше при неровной поверхности за счет данных гироскопа(он будет нужен, и с поддержкой I2C). Реализация по возможности :)

#define AX_12A 1

#define DYNAMIXEL_ROTATION_TO_UNIT 10.0
#define DEGREES_PER_SEC_2_RPM (1.0 / 6.0)

#define SERVO_TYPE AX_12A

#define CUM_DATA_SIZE_BYTES 16
#define CUM_DATA_OFFSET_BYTES 16
#define CUM_FPS 24
#define CUM_UPDATE_PERIOD_MS (1000 / CUM_FPS)

#define CUM_USE_KALMAN 1
#define CAM_KALMAN_ERR_MEASURE 5
#define CAM_KALMAN_Q 0.1
#define CAM_USE_KALMAN_FOR_CX 1
#define CAM_USE_KALMAN_FOR_CY 1
#define CAM_USE_KALMAN_FOR_AREA 1
#define CUM_OBJECTS_COUNT 2

#define ORIENT_UPDATE_PERIOD 20
#define ORIENT_ACC_TRESHOLD_VALUE 4000
#define ORIENT_USE_KALMAN_ACC 1
#define ORIENT_USE_KALMAN_GYRO 1
#define ORIENT_USE_KALMAN_MAG 1
#define ORIENT_KALMAN_ERR_MEASURE 500
#define ORIENT_KALMAN_Q 1


#define USE_RTOS 1 // если надо будет подключать openCM и у нее хватит памяти, то ставим 0, но этого скорее всего не будет