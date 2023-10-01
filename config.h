#pragma once

#define USE_CAMERA 1 //меняем на 0 если надо отключить модули (код для них не будет компилиться для экономии памяти)
#define USE_GYRO 1
#define USE_ACC 1

#define DEBUG_LOG 1
#define DEBUG_BOUD 115200

#define DEFAULT_BODY_Z_POZ 50 //высота туловища над полом
#define DEFAULT_BODY_X_OFFSET 0 //сдвиг туловища вперед для компенсации неустойчивости
#define DEFAULT_BODY_Y_ANGLE 0 //поворот туловища по оси Y для компенсации неустойчивости

#define AUTO_SURFACE_CALIBRATION 0 // автоматическое вычисление параметров выше при неровной поверхности за счет данных гироскопа(он будет нужен, и с поддержкой I2C). Реализация по возможности :)

#define SERVO_COUNT 6

#define AX_12A 1

#define SERVO_TYPE AX_12A

#define USE_RTOS 1 //если надо будет подключать openCM и у нее хватит памяти, то ставим 0, но этого скорее всего не будет