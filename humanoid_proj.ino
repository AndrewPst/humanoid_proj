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

    osThreadDef(THREAD_MAIN, thread_main, osPriorityNormal, 0, 8192 * 2);
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

std::vector<uint8_t> drivers;
// main function
static void thread_main(void const *arg)
{
    osDelay(3000);
    dynamixel::init(dynamixel::Dynamixel_config_t());
    dynamixel::checkConnections(&drivers, SERVO_COUNT);
    SERIAL_OUT_L_THRSAFE(drivers.size());
    osDelay(300);
    dynamixel::setupDrivers(drivers, 40, 0);
    std::vector<int32_t> angles(drivers.size(), 1023 / 2);
    angles[0] = RAD2VALUE(M_PI_4);

    dynamixel::syncWrite(drivers, angles, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
    // dynamixel::syncWrite({12, 14, 16}, speed, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);
    osDelay(5000);

    (void)arg;
    for (;;)
    {
        loop();
        if (serialEventRun)
            serialEventRun();
    }
}

kinematics::pos_t tpos;
kinematics::pos_t tposlast;
kinematics::leg_t _out;
double time_exec = 2;         // sec
const double speed_exec = 120; // mm/sec

#define EXE_MOD 1

const static kinematics::pos_t posarr[17]{
    kinematics::pos_t{0, 0, 140},
    kinematics::pos_t{50, 0, 140},
    kinematics::pos_t{50, 0, 140, 30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140, -30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140},
    kinematics::pos_t{50, 0, 140, 0, 30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140, 0, -30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140},
    kinematics::pos_t{50, 0, 140, 0, 0, 30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140, 0, 0, -30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140},
    kinematics::pos_t{50, 0, 140, 30 * DEG_TO_RAD, 0, 0},
    kinematics::pos_t{50, 0, 140, 30 * DEG_TO_RAD, 30 * DEG_TO_RAD, 0},
    kinematics::pos_t{50, 0, 140, 30 * DEG_TO_RAD, 30 * DEG_TO_RAD, -30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140, 0, 30 * DEG_TO_RAD, -30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140, 0, 0, -30 * DEG_TO_RAD},
    kinematics::pos_t{50, 0, 140, 0, 0, 0},
};

uint8_t exe_pos{0};

void loop()
{
#if DEBUG_LOG == 1 && EXE_MOD != 1

    bool avail{false};
    SERIAL_AVAIABLE_THRSAFE(avail);
    if (avail)
    {
        String strbuf;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.x = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.y = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.z = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.a = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.b = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.g = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_OUT_L_THRSAFE(tpos.x);
        SERIAL_OUT_L_THRSAFE(tpos.y);
        SERIAL_OUT_L_THRSAFE(tpos.z);
        SERIAL_OUT_L_THRSAFE(tpos.a);
        SERIAL_OUT_L_THRSAFE(tpos.b);
        kinematics::legIK(tpos, &_out);
        SERIAL_BEGIN;
        SERIAL_OUT(_out.values[0]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out.values[1]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out.values[2]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out.values[3]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out.values[4]);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(_out.values[5]);
        SERIAL_END;
        for (auto &i : _out.values)
        {
            if (isnan(i))
            {
                SERIAL_OUT_L_THRSAFE("NAN");
                return;
            }
        }
        std::vector<int32_t> pres_pos;
        dynamixel::syncReadPosition(drivers, pres_pos);
        time_exec = sqrt((tpos.x - tposlast.x) * (tpos.x - tposlast.x) + (tpos.y - tposlast.y) * (tpos.y - tposlast.y) + (tpos.z - tposlast.z) * (tpos.z - tposlast.z)) / speed_exec;
        if (time_exec < 1)
            time_exec = 1;
        SERIAL_OUT_L_THRSAFE(time_exec);
        dynamixel::syncWrite(drivers,
                             {
                                 (abs(_out.values[0] - VALUE2RAD(pres_pos[0]) + M_PI_4) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                 (abs(_out.values[1] - VALUE2RAD(pres_pos[1])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                 (abs(_out.values[2] - VALUE2RAD(pres_pos[2])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                 (abs(_out.values[3] - VALUE2RAD(pres_pos[3])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                 (abs(_out.values[4] - VALUE2RAD(pres_pos[4])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                 (abs(_out.values[5] - VALUE2RAD(pres_pos[5])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                             },
                             dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);
        osDelay(100);
        dynamixel::syncWrite(drivers,
                             {
                                 RAD2VALUE(_out.values[0] + M_PI_4),
                                 RAD2VALUE(_out.values[1]),
                                 RAD2VALUE(_out.values[2]),
                                 RAD2VALUE(_out.values[3]),
                                 RAD2VALUE(_out.values[4]),
                                 RAD2VALUE(_out.values[5]),
                             },
                             dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
        tposlast = tpos;
    }
    osDelay(5);
#elif EXE_MOD == 1

    tpos = posarr[exe_pos++];

    SERIAL_OUT_L_THRSAFE(tpos.x);
    SERIAL_OUT_L_THRSAFE(tpos.y);
    SERIAL_OUT_L_THRSAFE(tpos.z);
    SERIAL_OUT_L_THRSAFE(tpos.a);
    SERIAL_OUT_L_THRSAFE(tpos.b);
    SERIAL_OUT_L_THRSAFE(tpos.g);
    kinematics::legIK(tpos, &_out);
    // SERIAL_BEGIN;
    // SERIAL_OUT(_out.values[0]);
    // SERIAL_OUT('\t');
    // SERIAL_OUT(_out.values[1]);
    // SERIAL_OUT('\t');
    // SERIAL_OUT(_out.values[2]);
    // SERIAL_OUT('\t');
    // SERIAL_OUT(_out.values[3]);
    // SERIAL_OUT('\t');
    // SERIAL_OUT_L(_out.values[4]);
    // SERIAL_END;
    for (auto &i : _out.values)
    {
        if (isnan(i))
        {
            SERIAL_OUT_L_THRSAFE("NAN");
            return;
        }
    }
    std::vector<int32_t> pres_pos;
    dynamixel::syncReadPosition(drivers, pres_pos);
    time_exec = sqrt((tpos.x - tposlast.x) * (tpos.x - tposlast.x) + (tpos.y - tposlast.y) * (tpos.y - tposlast.y) + (tpos.z - tposlast.z) * (tpos.z - tposlast.z)) / speed_exec;
    if (time_exec < 1)
        time_exec = 0.4;
    SERIAL_OUT_L_THRSAFE(time_exec);
    dynamixel::syncWrite(drivers,
                         {
                             (abs(_out.values[0] - VALUE2RAD(pres_pos[0]) + M_PI_4) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                             (abs(_out.values[1] - VALUE2RAD(pres_pos[1])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                             (abs(_out.values[2] - VALUE2RAD(pres_pos[2])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                             (abs(_out.values[3] - VALUE2RAD(pres_pos[3])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                             (abs(_out.values[4] - VALUE2RAD(pres_pos[4])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                             (abs(_out.values[5] - VALUE2RAD(pres_pos[5])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                         },
                         dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);
    osDelay(100);
    dynamixel::syncWrite(drivers,
                         {
                             RAD2VALUE(_out.values[0] + M_PI_4),
                             RAD2VALUE(_out.values[1]),
                             RAD2VALUE(_out.values[2]),
                             RAD2VALUE(_out.values[3]),
                             RAD2VALUE(_out.values[4]),
                             RAD2VALUE(_out.values[5]),
                         },
                         dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
    tposlast = tpos;

    if (exe_pos >= 17)
        exe_pos = 0;

    osDelay(time_exec * 1000 + 50);
#endif
}
