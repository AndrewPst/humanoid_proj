#include <Arduino.h>
#include <RTOS.h>

#include "src/kinematics/kinematics.h"
#include "sout.h"
#include "src/dynamixel_drivers/servo.h"

#include <vector>

osMutexId _s_mutex = nullptr;
osThreadId thread_id_main;

void main_register_rtos()
{
    osMutexDef(_s_mutex);
    _s_mutex = osMutexCreate(osMutex(_s_mutex));

    osThreadDef(THREAD_MAIN, thread_main, osPriorityNormal, 0, 8192);
    thread_id_main = osThreadCreate(osThread(THREAD_MAIN), NULL);
}

void setup()
{
    // Register tasks
    main_register_rtos();
    // dynamixel::dynamixel_register_rtos();

    // SERIAL_INIT
    SERIAL_BEGIN
    SERIAL_INIT
    SERIAL_END

    // start kernel
    osKernelStart();
}

// main function
static void thread_main(void const *arg)
{
    osDelay(3000);
    std::vector<uint8_t> drivers;
    dynamixel::init(dynamixel::Dynamixel_config_t());
    dynamixel::checkConnections(&drivers, SERVO_COUNT);
    SERIAL_OUT_L_THRSAFE(drivers.size());
    osDelay(300);
    dynamixel::setupDrivers(drivers, 40, 0);
    std::vector<int32_t> angles(drivers.size(), 1023 / 2);

    dynamixel::syncWrite(drivers, angles, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
    // dynamixel::syncWrite({12, 14, 16}, speed, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);

    (void)arg;
    for (;;)
    {
        loop();
        if (serialEventRun)
            serialEventRun();
    }
}

String strbuf;
kinematics::pos_t tpos;
kinematics::leg_t _out;
const double time_exec = 2;
void loop()
{
    if (Serial.available())
    {
        SERIAL_READ_STRING_THREAD_SAVE(strbuf);
        tpos.z = strbuf.toFloat();
        SERIAL_OUT_L_THRSAFE(tpos.z);
        kinematics::legIK(tpos, &_out);
        SERIAL_BEGIN;
        SERIAL_OUT(_out.values[0]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out.values[1]);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(_out.values[2]);
        SERIAL_END;
        std::vector<int32_t> pres_pos;
        dynamixel::syncReadPosition({12, 14, 16}, pres_pos);
        dynamixel::syncWrite({12, 14, 16},
                             {(((abs(_out.values[0] - dynamixel::dxl.convertValue2Radian(pres_pos[0], 1024, 0, 5 * M_PI / 6.0, -5 * M_PI / 6.0)) * RAD_TO_DEG) / time_exec) / 6) * 10.0,
                              (((abs(_out.values[1] - dynamixel::dxl.convertValue2Radian(pres_pos[1], 1024, 0, 5 * M_PI / 6.0, -5 * M_PI / 6.0)) * RAD_TO_DEG) / time_exec) / 6) * 10.0,
                              (((abs(_out.values[2] - dynamixel::dxl.convertValue2Radian(pres_pos[2], 1024, 0, 5 * M_PI / 6.0, -5 * M_PI / 6.0)) * RAD_TO_DEG) / time_exec) / 6) * 10.0},
                             dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);
        osDelay(100);
        dynamixel::syncWrite({12, 14, 16},
                             {dynamixel::dxl.convertRadian2Value(_out.values[0], 1024, 0, 5 * M_PI / 6.0, -5 * M_PI / 6.0),
                              dynamixel::dxl.convertRadian2Value(_out.values[1], 1024, 0, 5 * M_PI / 6.0, -5 * M_PI / 6.0),
                              dynamixel::dxl.convertRadian2Value(_out.values[2], 1024, 0, 5 * M_PI / 6.0, -5 * M_PI / 6.0)},
                             dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
        // dxl_wb.goalPosition(indexes[0], (float)(_out._servos[0] + M_PI_2));
        // dxl_wb.goalPosition(indexes[1], (float)(_out._servos[1] + M_PI_2));
        // dxl_wb.goalPosition(indexes[2], (float)(M_PI_2 - _out._servos[2]));
    }
    osDelay(5);
}
