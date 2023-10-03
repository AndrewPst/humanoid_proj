#include <Arduino.h>
#include <RTOS.h>

#include "src/kinematics/kinematics.h"
#include "sout.h"
#include "src/dynamixel_drivers/servo.h"
#include "src/humanoid/humanoid.h"

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

std::vector<uint8_t> id_buff;

// main function
static void thread_main(void const *arg)
{
    osDelay(3000);
    humanoid::get_ids_of_limb(humanoid::LimbId::LIMB_LEG_LEFT, id_buff);
    dynamixel::init(dynamixel::Dynamixel_config_t());
    uint8_t count = dynamixel::checkConnections(id_buff);
    if (count == id_buff.size())
    {
        SERIAL_OUT_L_THRSAFE("Left leg init succ");
        // dynamixel::setupDrivers(id_buff, 20, 0);
    }
    else
    {
        SERIAL_OUT_L_THRSAFE("Left leg init err");
        while (1)
        {
            osDelay(10);
        }
    }
    humanoid::get_ids_of_limb(humanoid::LimbId::LIMB_LEG_RIGHT, id_buff);
    count = dynamixel::checkConnections(id_buff);
    if (count == id_buff.size())
    {
        SERIAL_OUT_L_THRSAFE("Right leg init succ");
        // dynamixel::setupDrivers(id_buff, 20, 0);
    }
    else
    {
        SERIAL_OUT_L_THRSAFE("Right leg init err");
        while (1)
        {
            osDelay(10);
        }
    }

    dynamixel::setupDrivers(std::vector<uint8_t>(humanoid::drivers_id_buffer().begin(), humanoid::drivers_id_buffer().end()), 20, 0);

    humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_LEFT, {RAD2VALUE(M_PI_4), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0)});
    humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_RIGHT, {RAD2VALUE(-M_PI_4), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0)});

    dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
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
const double speed_exec = 50; // mm/sec

void loop()
{
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
        SERIAL_OUT_L_THRSAFE(tpos.g);
        auto result = kinematics::legIK(tpos, &_out);
        SERIAL_BEGIN;
        SERIAL_OUT(_out[0]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out[1]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out[2]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out[3]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out[4]);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(_out[5]);
        SERIAL_END;
        if (result != kinematics::CalculationResult::CALC_SUCCESSFULL)
            return;
        auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_LEG_LEFT);
        dynamixel::syncReadPosition(humanoid::drivers_id_buffer().data()+l_range.first, (int32_t *)(humanoid::present_pos_buffer().data() + l_range.first), l_range.second);
        time_exec = sqrt((tpos.x - tposlast.x) * (tpos.x - tposlast.x) + (tpos.y - tposlast.y) * (tpos.y - tposlast.y) + (tpos.z - tposlast.z) * (tpos.z - tposlast.z)) / speed_exec;
        if (time_exec < 0.4)
            time_exec = 0.4;

        SERIAL_OUT_L_THRSAFE(time_exec);
        std::vector<int32_t> pres_pos;
        humanoid::get_present_pos_of_limb(humanoid::LimbId::LIMB_LEG_LEFT, pres_pos);
        humanoid::set_velocity_to_limb(humanoid::LimbId::LIMB_LEG_LEFT,
                                       {
                                           (abs(_out[0] - VALUE2RAD(pres_pos[0]) + M_PI_4) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                           (abs(_out[1] - VALUE2RAD(pres_pos[1])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                           (abs(_out[2] - VALUE2RAD(pres_pos[2])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                           (abs(_out[3] - VALUE2RAD(pres_pos[3])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                           (abs(_out[4] - VALUE2RAD(pres_pos[4])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                           (abs(_out[5] - VALUE2RAD(pres_pos[5])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
                                       });
        dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::velocity_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);

        humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_LEFT,
                                       {
                                           RAD2VALUE(_out[0] + M_PI_4),
                                           RAD2VALUE(_out[1]),
                                           RAD2VALUE(_out[2]),
                                           RAD2VALUE(_out[3]),
                                           RAD2VALUE(_out[4]),
                                           RAD2VALUE(_out[5]),
                                       });

        dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);

        // humanoid::get_ids_of_limb(humanoid::LimbId::LIMB_LEG_LEFT, id_buff);
        //  humanoid::set_goal_pos_to_limb(humanoid::LimbId)
        //  dynamixel::syncWrite(id_buff,
        //   {
        //       (abs(_out.values[0] - VALUE2RAD(pres_pos[0]) - M_PI_4) * RAD_TO_DEG) / time_exec / 6 * 10.0,
        //       (abs(_out.values[1] - VALUE2RAD(pres_pos[1])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
        //       (abs(_out.values[2] - VALUE2RAD(pres_pos[2])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
        //       (abs(_out.values[3] - VALUE2RAD(pres_pos[3])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
        //       (abs(_out.values[4] - VALUE2RAD(pres_pos[4])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
        //       (abs(_out.values[5] - VALUE2RAD(pres_pos[5])) * RAD_TO_DEG) / time_exec / 6 * 10.0,
        //   },
        //                      dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);
        // osDelay(100);
        // dynamixel::syncWrite(drivers,
        //  {
        //      RAD2VALUE(-_out.values[0] - M_PI_4),
        //      RAD2VALUE(-_out.values[1]),
        //      RAD2VALUE(-_out.values[2]),
        //      RAD2VALUE(-_out.values[3]),
        //      RAD2VALUE(-_out.values[4]),
        //      RAD2VALUE(-_out.values[5]),
        //  },
        //                      dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
        tposlast = tpos;
    }
    osDelay(5);
}
