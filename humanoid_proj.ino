#include <Arduino.h>
#include <RTOS.h>

#include "src/kinematics/kinematics.h"
#include "sout.h"
#include "src/dynamixel_drivers/dynamixel.h"
#include "src/humanoid/humanoid.h"
#include "src/lidar/lidar.h"
#include "src/camera/camera.h"
#include "src/orientation/orientation.h"

#include <vector>

osMutexId _s_mutex = nullptr;
osThreadId thread_id_main;

enum InitResult : uint8_t
{
    INIT_SUCC = 0,
    INIT_LEFT_LEG_ERROR_PING,
    INIT_RIGHT_LEG_ERROR_PING,
    INIT_LEFT_HAND_ERROR_PING,
    INIT_RIGHT_HAND_ERROR_PING,
    INIT_SETUP_DRIVERS_ERROR,
    INIT_DYNAMIXEL_DRIVER_ERROR,
    INIT_CAMERA_NOT_FOUND_ERROR,
    INIT_ORIENT_ERROR,
};

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
    humanoid::humanoid_register_rtos();
    camera::camera_register_rtos();
    // dynamixel::dynamixel_register_rtos();

    // SERIAL_INIT
    SERIAL_BEGIN
    SERIAL_INIT
    SERIAL_END

    // start kernel
    osKernelStart();
}

InitResult initsystem()
{
    kinematics::set_max_values_legs_joints(humanoid::get_max_angles_of_limb(humanoid::LimbId::LIMB_LEG_LEFT));
    kinematics::set_min_values_legs_joints(humanoid::get_min_angles_of_limb(humanoid::LimbId::LIMB_LEG_LEFT));
    kinematics::set_min_values_hands_joints(humanoid::get_min_angles_of_limb(humanoid::LimbId::LIMB_HAND_LEFT));
    kinematics::set_max_values_hands_joints(humanoid::get_max_angles_of_limb(humanoid::LimbId::LIMB_HAND_LEFT));

    if (dynamixel::init(dynamixel::Dynamixel_config_t()) != 0)
        return InitResult::INIT_DYNAMIXEL_DRIVER_ERROR;

    // const humanoid::LimbId limbs[humanoid::LIMB_COUNT]{humanoid::LIMB_LEG_LEFT, humanoid::LIMB_LEG_RIGHT, humanoid::LIMB_HAND_LEFT, humanoid::LIMB_HAND_RIGHT, humanoid::LIMB_HEAD};

    // uint8_t drivers_count{0};
    // std::vector<uint8_t> id_buff;
    // for (uint8_t i = 0; i < humanoid::LIMB_COUNT; ++i)
    // {
    //     humanoid::get_ids_of_limb(limbs[i], id_buff);
    //     drivers_count = dynamixel::checkConnections(id_buff);
    //     if (drivers_count != id_buff.size())
    //     {
    //         return static_cast<InitResult>(i + 1);
    //     }
    // }

    // drivers_count = dynamixel::setupDrivers(std::vector<uint8_t>(humanoid::drivers_id_buffer().begin(), humanoid::drivers_id_buffer().end()), SERVO_COUNT);
    // if (drivers_count != SERVO_COUNT)
    //     return InitResult::INIT_SETUP_DRIVERS_ERROR;

    // auto r = dynamixel::checkCamera();
    auto r = orientation::init();
    if (r)
        return InitResult::INIT_ORIENT_ERROR;

    return InitResult::INIT_SUCC;
}

// main function
static void thread_main(void const *arg)
{
    (void)arg;

    auto r = initsystem();
    if (r != INIT_SUCC)
    {
        SERIAL_BEGIN;
        SERIAL_OUT("[CORE] Init system error;\tCode: ");
        SERIAL_OUT_L((int)r);
        SERIAL_END;
        while (1)
        {
            osDelay(50);
        }
    }
    // humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_LEFT, {RAD2VALUE(M_PI_4), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0)});
    // humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_RIGHT, {RAD2VALUE(-M_PI_4), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0)});
    // humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_HAND_LEFT, {RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0)});
    // humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_HAND_RIGHT, {RAD2VALUE(0), RAD2VALUE(0), RAD2VALUE(0)});
    // humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_HEAD, {RAD2VALUE(0), RAD2VALUE(0)});

    // dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
    // osDelay(5000);
    SERIAL_OUT_L_THRSAFE("Init succ");
    for (;;)
    {
        loop();
        SERIAL_BEGIN
        if (serialEventRun)
            serialEventRun();
        SERIAL_END
    }
}

#pragma region ACC get push
#if 0

orientation::Orient_vector_t acc1;
orientation::Orient_vector_t acc2;

void loop()
{
    delay(ORIENT_UPDATE_PERIOD);
    orientation::update();
    orientation::get_acc_vector(acc1);
    delay(ORIENT_UPDATE_PERIOD);
    orientation::update();
    orientation::get_acc_vector(acc2);
    auto r = orientation::acc_push_recognize(acc1, acc2);

    if (r.lenght() == 0)
        return;
    SERIAL_BEGIN

    if (abs(r.x) > 0)
    {
        SERIAL_OUT("X: ");
        SERIAL_OUT(r.x);
        SERIAL_OUT('\t');
    }
    else
    {
        SERIAL_OUT("\t\t")
    }
    if (abs(r.y) > 0)
    {
        SERIAL_OUT("Y: ");
        SERIAL_OUT(r.y);
        SERIAL_OUT('\t');
    }
    else
    {
        SERIAL_OUT("\t\t")
    }
    if (abs(r.z) > 0)
    {
        SERIAL_OUT("Z: ");
        SERIAL_OUT(r.z);
        SERIAL_OUT('\t');
    }
    else
    {
        SERIAL_OUT("\t\t")
    }
    SERIAL_OUT("Forse: ");
    SERIAL_OUT_L(r.lenght());
    SERIAL_END;
}

#endif
#pragma endregion

#pragma region camera
#if 0

camera::CumObjectMetadata_t object;
void loop()
{
    dynamixel::readCumRegister(0, object.buffer.data());
    camera::filter_data(object);
    SERIAL_BEGIN;
    SERIAL_OUT(object.metadata_args.type);
    SERIAL_OUT('\t');
    SERIAL_OUT(object.metadata_args.cx);
    SERIAL_OUT('\t');
    SERIAL_OUT(object.metadata_args.cy);
    SERIAL_OUT('\t');
    SERIAL_OUT(object.metadata_args.area);
    SERIAL_OUT('\t');
    SERIAL_OUT(object.metadata_args.left);
    SERIAL_OUT('\t');
    SERIAL_OUT(object.metadata_args.right);
    SERIAL_OUT('\t');
    SERIAL_OUT(object.metadata_args.top);
    SERIAL_OUT('\t');
    SERIAL_OUT_L(object.metadata_args.bottom);
    SERIAL_END;
    osDelay(500);
}

#endif
#pragma endregion

#pragma region hand ik
#if 0

kinematics::pos_cylindrical_t tpos;
kinematics::pos_cylindrical_t tposlast;
kinematics::hand_t _out_left;
kinematics::hand_t _out_right;
double time_exec = 2;         // sec
const double speed_exec = 40; // mm/sec

void loop()
{
    bool avail{false};
    SERIAL_AVAIABLE_THRSAFE(avail);
    if (avail)
    {
        String strbuf;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.a = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.r = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        tpos.z = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        int v = strbuf.toInt();

        SERIAL_BEGIN
        SERIAL_OUT(tpos.a);
        SERIAL_OUT('\t');
        SERIAL_OUT(tpos.r);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(tpos.z);
        SERIAL_END

        kinematics::IKCalcConfig conf = (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED |
                                                                   kinematics::IKCONF_CHECK_UNREACHABLE_COORDS);
        if (v)
            conf = (kinematics::IKCalcConfig)(conf | kinematics::IKCONFIG_USE_LEFT_HAND_COOR_SYSTEM);

        auto result = kinematics::handIK(tpos, &_out_left, conf);
        result = kinematics::handIK(tpos, &_out_right, (kinematics::IKCalcConfig)(conf | kinematics::IKCONFIG_MIRROR_OUT));

        SERIAL_BEGIN;
        SERIAL_OUT(_out_left[0]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out_left[1]);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(_out_left[2]);
        SERIAL_OUT(_out_right[0]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out_right[1]);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(_out_right[2]);
        SERIAL_END;

        if (result != kinematics::CalculationResult::CALC_SUCCESSFULL)
            return;

        std::array<double, HAND_DRIVERS_COUNT> pres_pos;
        std::vector<int32_t> new_values(HAND_DRIVERS_COUNT, 0);

        //========Calc execution time==========
        time_exec = kinematics::get_motion_time_by_speed(tposlast, tpos, speed_exec);
        if (time_exec == 0)
            time_exec = 3;
        SERIAL_OUT_L_THRSAFE(time_exec);

        //=======Calc drivers velocity in left hand=======
        auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_HAND_LEFT);
        dynamixel::syncReadPosition(humanoid::drivers_id_buffer().data() + l_range.first, (int32_t *)(humanoid::present_pos_buffer().data() + l_range.first), l_range.second);

        dynamixel::value_to_rad_arr((int32_t *)(humanoid::present_pos_buffer().data() + l_range.first),
                                    pres_pos.data(), l_range.second,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_LEFT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_LEFT));

        kinematics::calc_joint_velocity_by_time_arr(pres_pos.data(), _out_left.data(), new_values.data(), HAND_DRIVERS_COUNT, time_exec);
        humanoid::set_velocity_to_limb(humanoid::LimbId::LIMB_HAND_LEFT, new_values);

        dynamixel::syncWrite(humanoid::drivers_id_buffer().data() + l_range.first, humanoid::velocity_buffer().data() + l_range.first, l_range.second, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);

        //=======Calc drivers velocity in right hand=======
        l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_HAND_RIGHT);
        dynamixel::syncReadPosition(humanoid::drivers_id_buffer().data() + l_range.first, (int32_t *)(humanoid::present_pos_buffer().data() + l_range.first), l_range.second);

        dynamixel::value_to_rad_arr((int32_t *)(humanoid::present_pos_buffer().data() + l_range.first),
                                    pres_pos.data(), l_range.second,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_RIGHT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_RIGHT));

        kinematics::calc_joint_velocity_by_time_arr(pres_pos.data(), _out_left.data(), new_values.data(), HAND_DRIVERS_COUNT, time_exec);
        humanoid::set_velocity_to_limb(humanoid::LimbId::LIMB_HAND_RIGHT, new_values);

        dynamixel::syncWrite(humanoid::drivers_id_buffer().data() + l_range.first, humanoid::velocity_buffer().data() + l_range.first, l_range.second, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);

        //=========Send to drivers=======
        dynamixel::rad_to_value_arr(_out_left.data(), new_values.data(), new_values.size(), humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_LEFT), humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_LEFT));
        humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_HAND_LEFT, new_values);

        dynamixel::rad_to_value_arr(_out_right.data(), new_values.data(), new_values.size(), humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_RIGHT), humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_RIGHT));
        humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_HAND_RIGHT, new_values);

        dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);

        tposlast = tpos;
    }
    osDelay(5);
}

#endif

#pragma endregion

#pragma region legs ik
#if 0
kinematics::pos_t tpos;
kinematics::pos_t tposlast;
kinematics::leg_t _out_left;
kinematics::leg_t _out_right;
double time_exec = 2;         // sec
const double speed_exec = 40; // mm/sec

void loop()
{
    bool avail{false};
    SERIAL_AVAIABLE_THRSAFE(avail);
    if (avail)
    {
        //=======Read serial=========
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
        SERIAL_BEGIN;
        SERIAL_OUT(tpos.x);
        SERIAL_OUT('\t');
        SERIAL_OUT(tpos.y);
        SERIAL_OUT('\t');
        SERIAL_OUT(tpos.z);
        SERIAL_OUT('\t');
        SERIAL_OUT(tpos.a);
        SERIAL_OUT('\t');
        SERIAL_OUT(tpos.b);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(tpos.g);
        SERIAL_END;

        //=======Calc ik===========
        auto result = kinematics::legIK(tpos, &_out_left,
                                        (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED |
                                                                   kinematics::IKCONF_CHECK_UNREACHABLE_COORDS));
        result = kinematics::legIK(tpos, &_out_right,
                                   (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED |
                                                              kinematics::IKCONF_CHECK_UNREACHABLE_COORDS |
                                                              kinematics::IKCONFIG_MIRROR_OUT));
        SERIAL_BEGIN;
        SERIAL_OUT(_out_right[0]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out_right[1]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out_right[2]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out_right[3]);
        SERIAL_OUT('\t');
        SERIAL_OUT(_out_right[4]);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(_out_right[5]);
        SERIAL_END;
        if (result != kinematics::CalculationResult::CALC_SUCCESSFULL)
            return;

        //========Read current pos=======
        dynamixel::syncReadPosition(humanoid::drivers_id_buffer().data(), (int32_t *)(humanoid::present_pos_buffer().data()), SERVO_COUNT);

        //==========Calc moving velocity=========
        time_exec = kinematics::get_motion_time_by_speed(tposlast, tpos, speed_exec);

        //==Left leg==
        SERIAL_OUT_L_THRSAFE(time_exec);
        std::array<double, LEG_DRIVERS_COUNT> pres_pos;

        auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_LEG_LEFT);
        dynamixel::value_to_rad_arr((int32_t *)(humanoid::present_pos_buffer().data() + l_range.first),
                                    pres_pos.data(), l_range.second,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_LEFT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_LEFT));

        std::vector<int32_t> new_values(LEG_DRIVERS_COUNT, 0);
        kinematics::calc_joint_velocity_by_time_arr(pres_pos.data(), _out_left.data(), new_values.data(), LEG_DRIVERS_COUNT, time_exec);
        humanoid::set_velocity_to_limb(humanoid::LimbId::LIMB_LEG_LEFT, new_values);
        SERIAL_BEGIN;
        SERIAL_OUT(new_values[0]);
        SERIAL_OUT('\t');
        SERIAL_OUT(new_values[1]);
        SERIAL_OUT('\t');
        SERIAL_OUT(new_values[2]);
        SERIAL_OUT('\t');
        SERIAL_OUT(new_values[3]);
        SERIAL_OUT('\t');
        SERIAL_OUT(new_values[4]);
        SERIAL_OUT('\t');
        SERIAL_OUT_L(new_values[5]);
        SERIAL_END;

        //==Right leg==
        l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_LEG_RIGHT);
        dynamixel::value_to_rad_arr((int32_t *)(humanoid::present_pos_buffer().data() + l_range.first),
                                    pres_pos.data(), l_range.second,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_RIGHT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_RIGHT));

        kinematics::calc_joint_velocity_by_time_arr(pres_pos.data(), _out_right.data(), new_values.data(), LEG_DRIVERS_COUNT, time_exec);
        humanoid::set_velocity_to_limb(humanoid::LimbId::LIMB_LEG_RIGHT, new_values);

        dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::velocity_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);

        //============Set goal pos============
        dynamixel::rad_to_value_arr(_out_left.data(), new_values.data(), new_values.size(), humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_LEFT), humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_LEFT));
        humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_LEFT, new_values);
        dynamixel::rad_to_value_arr(_out_right.data(), new_values.data(), new_values.size(), humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_RIGHT), humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_RIGHT));
        humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_RIGHT, new_values);

        dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);

        tposlast = tpos;
    }
    osDelay(5);
}
#endif
#pragma endregion