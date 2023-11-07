#include "trajectory.h"
#include "../dynamixel_drivers/dynamixel.h"

#include <atomic>

// #include <stdatomic.h>

namespace trajectory
{
    TrajectoryLegsTimeline_t _leg_left_timeline, _leg_right_timeline;
    TrajectoryHandsTimeline_t _hand_left_timeline, _hand_right_timeline;

    std::shared_ptr<GoalPointerLeg_t> _current_leg_left;
    std::shared_ptr<GoalPointerLeg_t> _current_leg_right;
    std::shared_ptr<GoalPointerHand_t> _current_hand_left;
    std::shared_ptr<GoalPointerHand_t> _current_hand_right;

    kinematics::pos_t _started_l_l;
    kinematics::pos_t _started_l_r;
    kinematics::pos_cylindrical_t _started_h_l;
    kinematics::pos_cylindrical_t _started_h_r;

    uint32_t _l_l_exec_start_time{0};
    uint32_t _l_r_exec_start_time{0};
    uint32_t _h_l_exec_start_time{0};
    uint32_t _h_r_exec_start_time{0};

    GoalPointerLeg_t *_executable_leg_left{nullptr};
    GoalPointerLeg_t *_executable_leg_right{nullptr};
    GoalPointerHand_t *_executable_hand_left{nullptr};
    GoalPointerHand_t *_executable_hand_right{nullptr};

    osMutexId _l_l_timeline_mutex{nullptr};

    osMutexId _l_r_timeline_mutex{nullptr};

    osMutexId _h_l_timeline_mutex{nullptr};

    osMutexId _h_r_timeline_mutex{nullptr};

    osMutexId _l_l_exec_mutex{nullptr};

    osMutexId _l_r_exec_mutex{nullptr};

    osMutexId _h_l_exec_mutex{nullptr};

    osMutexId _h_r_exec_mutex{nullptr};

    std::atomic<bool> _pause_l_l;
    std::atomic<bool> _pause_l_r;
    std::atomic<bool> _pause_h_l;
    std::atomic<bool> _pause_h_r;

    union
    {
        struct
        {
            std::array<double, LEG_DRIVERS_COUNT> l_l;
            std::array<double, LEG_DRIVERS_COUNT> l_r;
            std::array<double, HAND_DRIVERS_COUNT> h_l;
            std::array<double, HAND_DRIVERS_COUNT> h_r;
            std::array<double, HEAD_DRIVERS_COUNT> head;
        } individual_buffers;
        std::array<double, SERVO_COUNT> general_buf;
    } _ik_rads_buff;

    union
    {
        struct
        {
            std::array<double, LEG_DRIVERS_COUNT> l_l;
            std::array<double, LEG_DRIVERS_COUNT> l_r;
            std::array<double, HAND_DRIVERS_COUNT> h_l;
            std::array<double, HAND_DRIVERS_COUNT> h_r;
            std::array<double, HEAD_DRIVERS_COUNT> head;
        } individual_buffers;
        std::array<double, SERVO_COUNT> general_buf;
    } _last_rads_buff;

    const std::array<double, SERVO_COUNT> &get_last_rads_buff()
    {
        return _last_rads_buff.general_buf;
    }

    const std::array<double, SERVO_COUNT> &get_ik_rads_buff()
    {
        return _ik_rads_buff.general_buf;
    }

    uint32_t c_time{0};

    const uint32_t UPDATE_PERIOD = 10;

    void func_leg_left()
    {
        kinematics::pos_t n_pos;
        _current_leg_left->lock_time_mutex();
        uint32_t cur_time = *(_current_leg_left->exec_time) = c_time - _l_l_exec_start_time;
        _current_leg_left->unlock_time_mutex();

        _executable_leg_left->lock_time_mutex();
        uint32_t ex_time = *(_executable_leg_left->exec_time);
        _executable_leg_left->unlock_time_mutex();

        double time = (double)UPDATE_PERIOD / 1000.0;

        // SERIAL_OUT_L_THRSAFE(cur_time);
        if (cur_time + UPDATE_PERIOD >= ex_time)
        {
            // SERIAL_OUT_L_THRSAFE("Ended");
            _executable_leg_left->lock_pos_mutex();
            n_pos = *(_executable_leg_left->pos);
            _executable_leg_left->unlock_pos_mutex();
            osMutexWait(_l_l_exec_mutex, 0);
            _executable_leg_left = NULL;
            osMutexRelease(_l_l_exec_mutex);
            time = (double)ex_time - (double)cur_time;
        }
        else
        {
            double linear_koef = (double)((c_time + UPDATE_PERIOD) - (_l_l_exec_start_time)) / (double)(ex_time);

            _executable_leg_left->lock_pos_mutex();
            n_pos.x = _started_l_l.x + (_executable_leg_left->pos->x - _started_l_l.x) * linear_koef;
            n_pos.y = _started_l_l.y + (_executable_leg_left->pos->y - _started_l_l.y) * linear_koef;
            n_pos.z = _started_l_l.z + (_executable_leg_left->pos->z - _started_l_l.z) * linear_koef;
            n_pos.a = _started_l_l.a + (_executable_leg_left->pos->a - _started_l_l.a) * linear_koef;
            n_pos.b = _started_l_l.b + (_executable_leg_left->pos->b - _started_l_l.b) * linear_koef;
            n_pos.g = _started_l_l.g + (_executable_leg_left->pos->g - _started_l_l.g) * linear_koef;
            _executable_leg_left->unlock_pos_mutex();
        }
        // SERIAL_BEGIN;
        // SERIAL_OUT_L("LEFT:");
        // SERIAL_OUT("X=");
        // SERIAL_OUT(n_pos.x);
        // SERIAL_OUT("\tY=");
        // SERIAL_OUT(n_pos.y);
        // SERIAL_OUT("\tZ=");
        // SERIAL_OUT(n_pos.z);
        // SERIAL_OUT("\tA=");
        // SERIAL_OUT(n_pos.a);
        // SERIAL_OUT("\tB=");
        // SERIAL_OUT(n_pos.b);
        // SERIAL_OUT("\tG=");
        // SERIAL_OUT_L(n_pos.g);
        // SERIAL_END;

        auto r = kinematics::legIK(n_pos, &(_ik_rads_buff.individual_buffers.l_l), (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS));
        if (r != kinematics::CalculationResult::CALC_SUCCESSFULL)
        {
            SERIAL_OUT_THRSAFE("[MOTION_ENGINE] IK L_L err code:\t");
            SERIAL_OUT_L_THRSAFE(r);
            return;
        }

        auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LIMB_LEG_LEFT);

        dynamixel::values_backlash_compensate_rad(_ik_rads_buff.individual_buffers.l_l.data(),
                                                  _ik_rads_buff.individual_buffers.l_l.data(), LEG_DRIVERS_COUNT,
                                                  dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);
        humanoid::lock_velocity_buffer_mutex();

        kinematics::calc_joint_velocity_by_time_arr(_last_rads_buff.individual_buffers.l_l.data(),
                                                    _ik_rads_buff.individual_buffers.l_l.data(),
                                                    (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                    LEG_DRIVERS_COUNT, time);
        humanoid::unlock_velocity_buffer_mutex();

        humanoid::lock_goal_pos_buffer_mutex();
        dynamixel::rad_to_value_arr(_ik_rads_buff.individual_buffers.l_l.data(),
                                    (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first),
                                    LEG_DRIVERS_COUNT,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_LEFT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_LEFT));
        humanoid::unlock_goal_pos_buffer_mutex();

        _last_rads_buff.individual_buffers.l_l = _ik_rads_buff.individual_buffers.l_l;
        _current_leg_left->lock_pos_mutex();
        *(_current_leg_left->pos) = n_pos;
        _current_leg_left->unlock_pos_mutex();
    }

    void func_leg_right()
    {
        kinematics::pos_t n_pos;

        _current_leg_right->lock_time_mutex();
        uint32_t cur_time = *(_current_leg_right->exec_time) = c_time - _l_r_exec_start_time;
        _current_leg_right->unlock_time_mutex();

        _executable_leg_right->lock_time_mutex();
        uint32_t ex_time = *(_executable_leg_right->exec_time);
        _executable_leg_right->unlock_time_mutex();

        double time = (double)UPDATE_PERIOD / 1000.0;

        if (cur_time + UPDATE_PERIOD >= ex_time)
        {
            _executable_leg_right->lock_pos_mutex();
            n_pos = *(_executable_leg_right->pos);
            _executable_leg_right->unlock_pos_mutex();
            osMutexWait(_l_r_exec_mutex, 0);
            _executable_leg_right = NULL;
            osMutexRelease(_l_r_exec_mutex);
            time = (double)ex_time - (double)cur_time;
        }
        else
        {
            double linear_koef = (double)((c_time + UPDATE_PERIOD) - (_l_r_exec_start_time)) / (double)(ex_time);

            _executable_leg_right->lock_pos_mutex();
            n_pos.x = _started_l_r.x + (_executable_leg_right->pos->x - _started_l_r.x) * linear_koef;
            n_pos.y = _started_l_r.y + (_executable_leg_right->pos->y - _started_l_r.y) * linear_koef;
            n_pos.z = _started_l_r.z + (_executable_leg_right->pos->z - _started_l_r.z) * linear_koef;
            n_pos.a = _started_l_r.a + (_executable_leg_right->pos->a - _started_l_r.a) * linear_koef;
            n_pos.b = _started_l_r.b + (_executable_leg_right->pos->b - _started_l_r.b) * linear_koef;
            n_pos.g = _started_l_r.g + (_executable_leg_right->pos->g - _started_l_r.g) * linear_koef;
            // SERIAL_BEGIN;
            // SERIAL_OUT_L("RIGHT:");
            // SERIAL_OUT("X=");
            // SERIAL_OUT(n_pos.x);
            // SERIAL_OUT("\tY=");
            // SERIAL_OUT(n_pos.y);
            // SERIAL_OUT("\tZ=");
            // SERIAL_OUT(n_pos.z);
            // SERIAL_OUT("\tA=");
            // SERIAL_OUT(n_pos.a);
            // SERIAL_OUT("\tB=");
            // SERIAL_OUT(n_pos.b);
            // SERIAL_OUT("\tG=");
            // SERIAL_OUT_L(n_pos.g);
            // SERIAL_END;
            _executable_leg_right->unlock_pos_mutex();
        }

        auto r = kinematics::legIK(n_pos, &(_ik_rads_buff.individual_buffers.l_r), (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS | kinematics::IKCONFIG_MIRROR_OUT));
        if (r != kinematics::CalculationResult::CALC_SUCCESSFULL)
        {
            SERIAL_OUT_THRSAFE("[MOTION_ENGINE] IK L_R err code:\t");
            SERIAL_OUT_L_THRSAFE(r);
            return;
        }

        auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LIMB_LEG_RIGHT);

        dynamixel::values_backlash_compensate_rad(_ik_rads_buff.individual_buffers.l_r.data(),
                                                  _ik_rads_buff.individual_buffers.l_r.data(), LEG_DRIVERS_COUNT,
                                                  dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);

        humanoid::lock_velocity_buffer_mutex();
        kinematics::calc_joint_velocity_by_time_arr(_last_rads_buff.individual_buffers.l_r.data(),
                                                    _ik_rads_buff.individual_buffers.l_r.data(),
                                                    (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                    LEG_DRIVERS_COUNT, time);
        humanoid::unlock_velocity_buffer_mutex();

        humanoid::lock_goal_pos_buffer_mutex();
        dynamixel::rad_to_value_arr(_ik_rads_buff.individual_buffers.l_r.data(),
                                    (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first),
                                    LEG_DRIVERS_COUNT,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_RIGHT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_RIGHT));
        humanoid::unlock_goal_pos_buffer_mutex();

        _last_rads_buff.individual_buffers.l_r = _ik_rads_buff.individual_buffers.l_r;
        _current_leg_right->lock_pos_mutex();
        *(_current_leg_right->pos) = n_pos;
        _current_leg_right->unlock_pos_mutex();
    }

    void func_left_hand()
    {
        kinematics::pos_cylindrical_t n_pos;

        _current_hand_left->lock_time_mutex();
        uint32_t cur_time = *(_current_hand_left->exec_time) = c_time - _h_l_exec_start_time;
        _current_hand_left->unlock_time_mutex();

        _executable_hand_left->lock_time_mutex();
        uint32_t ex_time = *(_executable_hand_left->exec_time);
        _executable_hand_left->unlock_time_mutex();

        double time = (double)UPDATE_PERIOD / 1000.0;

        if (cur_time + UPDATE_PERIOD >= ex_time)
        {
            _executable_hand_left->lock_pos_mutex();
            n_pos = *(_executable_hand_left->pos);
            _executable_hand_left->unlock_pos_mutex();
            osMutexWait(_h_l_exec_mutex, 0);
            _executable_hand_left = NULL;
            osMutexRelease(_h_l_exec_mutex);
            time = (double)ex_time - (double)cur_time;
        }
        else
        {

            double linear_koef = (double)((c_time + UPDATE_PERIOD) - (_h_l_exec_start_time)) / (double)(ex_time);

            _executable_hand_left->lock_pos_mutex();
            n_pos.a = _started_h_l.a + (_executable_hand_left->pos->a - _started_h_l.a) * linear_koef;
            n_pos.r = _started_h_l.r + (_executable_hand_left->pos->r - _started_h_l.r) * linear_koef;
            n_pos.z = _started_h_l.z + (_executable_hand_left->pos->z - _started_h_l.z) * linear_koef;
            _executable_hand_left->unlock_pos_mutex();
        }

        auto r = kinematics::handIK(n_pos, &(_ik_rads_buff.individual_buffers.h_l), (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS | kinematics::IKCONFIG_MIRROR_OUT));
        if (r != kinematics::CalculationResult::CALC_SUCCESSFULL)
        {
            SERIAL_OUT_THRSAFE("[MOTION_ENGINE] IK H_L err code:\t");
            SERIAL_OUT_L_THRSAFE(r);
            return;
        }

        auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LIMB_HAND_LEFT);

        dynamixel::values_backlash_compensate_rad(_ik_rads_buff.individual_buffers.h_l.data(),
                                                  _ik_rads_buff.individual_buffers.h_l.data(), HAND_DRIVERS_COUNT,
                                                  dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);

        humanoid::lock_velocity_buffer_mutex();
        kinematics::calc_joint_velocity_by_time_arr(_last_rads_buff.individual_buffers.h_l.data(),
                                                    _ik_rads_buff.individual_buffers.h_l.data(),
                                                    (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                    HAND_DRIVERS_COUNT, time);
        humanoid::unlock_velocity_buffer_mutex();

        humanoid::lock_goal_pos_buffer_mutex();
        dynamixel::rad_to_value_arr(_ik_rads_buff.individual_buffers.h_l.data(),
                                    (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first),
                                    HAND_DRIVERS_COUNT,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_LEFT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_LEFT));
        humanoid::unlock_goal_pos_buffer_mutex();

        _last_rads_buff.individual_buffers.h_l = _ik_rads_buff.individual_buffers.h_l;
        _current_hand_left->lock_pos_mutex();
        *(_current_hand_left->pos) = n_pos;
        _current_hand_left->unlock_pos_mutex();
    }

    void func_right_hand()
    {
        kinematics::pos_cylindrical_t n_pos;

        _current_hand_right->lock_time_mutex();
        uint32_t cur_time = *(_current_hand_right->exec_time) = c_time - _h_r_exec_start_time;
        _current_hand_right->unlock_time_mutex();

        _executable_hand_right->lock_time_mutex();
        uint32_t ex_time = *(_executable_hand_right->exec_time);
        _executable_hand_right->unlock_time_mutex();

        double time = (double)UPDATE_PERIOD / 1000.0;

        if (cur_time + UPDATE_PERIOD >= ex_time)
        {
            // SERIAL_OUT_L_THRSAFE("END");
            _executable_hand_right->lock_pos_mutex();
            n_pos = *(_executable_hand_right->pos);
            _executable_hand_right->unlock_pos_mutex();
            osMutexWait(_h_r_exec_mutex, 0);
            _executable_hand_right = NULL;
            osMutexRelease(_h_r_exec_mutex);
            time = (double)ex_time - (double)cur_time;
        }
        else
        {
            // SERIAL_OUT_L_THRSAFE("Calc");
            double linear_koef = (double)((c_time + UPDATE_PERIOD) - (_h_r_exec_start_time)) / (double)(ex_time);

            _executable_hand_right->lock_pos_mutex();
            n_pos.a = _started_h_r.a + (_executable_hand_right->pos->a - _started_h_r.a) * linear_koef;
            n_pos.r = _started_h_r.r + (_executable_hand_right->pos->r - _started_h_r.r) * linear_koef;
            n_pos.z = _started_h_r.z + (_executable_hand_right->pos->z - _started_h_r.z) * linear_koef;
            _executable_hand_right->unlock_pos_mutex();
        }

        auto r = kinematics::handIK(n_pos, &(_ik_rads_buff.individual_buffers.h_r), (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS));
        if (r != kinematics::CalculationResult::CALC_SUCCESSFULL)
        {
            SERIAL_OUT_THRSAFE("[MOTION_ENGINE] IK H_R err code:\t");
            SERIAL_OUT_L_THRSAFE(r);
            return;
        }

        auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LIMB_HAND_RIGHT);

        dynamixel::values_backlash_compensate_rad(_ik_rads_buff.individual_buffers.h_r.data(),
                                                  _ik_rads_buff.individual_buffers.h_r.data(), HAND_DRIVERS_COUNT,
                                                  dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);

        humanoid::lock_velocity_buffer_mutex();
        kinematics::calc_joint_velocity_by_time_arr(_last_rads_buff.individual_buffers.h_r.data(),
                                                    _ik_rads_buff.individual_buffers.h_r.data(),
                                                    (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                    HAND_DRIVERS_COUNT, time);
        humanoid::unlock_velocity_buffer_mutex();

        humanoid::lock_goal_pos_buffer_mutex();
        dynamixel::rad_to_value_arr(_ik_rads_buff.individual_buffers.h_r.data(),
                                    (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first),
                                    HAND_DRIVERS_COUNT,
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_RIGHT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_RIGHT));
        humanoid::unlock_goal_pos_buffer_mutex();

        _last_rads_buff.individual_buffers.h_r = _ik_rads_buff.individual_buffers.h_r;
        _current_hand_right->lock_pos_mutex();
        *(_current_hand_right->pos) = n_pos;
        _current_hand_right->unlock_pos_mutex();
    }

    void thread_func(void const *)
    {
        uint32_t _l_l_last_calc_time{0},
            _l_r_last_calc_time{0},
            _h_l_last_calc_time{0},
            _h_r_last_calc_time{0};

        osDelay(5000);
        SERIAL_OUT_L_THRSAFE("Traj thread started");
        for (;;)
        {
            c_time = millis();
            bool _update = false;

            // LEFT LEG calc

            if (!_executable_leg_left) // if is nullptr
            {
                osMutexWait(_l_l_timeline_mutex, portMAX_DELAY);
                bool emp = _leg_left_timeline.empty();
                osMutexRelease(_l_l_timeline_mutex);
                if (emp == false)
                {
                    osMutexWait(_l_l_exec_mutex, portMAX_DELAY);
                    osMutexWait(_l_l_timeline_mutex, portMAX_DELAY);
                    _executable_leg_left = _leg_left_timeline.front();
                    osMutexRelease(_l_l_exec_mutex);
                    _leg_left_timeline.pop();
                    osMutexRelease(_l_l_timeline_mutex);

                    _current_leg_left->lock_time_mutex();
                    *(_current_leg_left->exec_time) = 0;
                    _current_leg_left->unlock_time_mutex();

                    _l_l_exec_start_time = c_time;
                    _started_l_l = *(_current_leg_left->pos);
                    // SERIAL_BEGIN;
                    // SERIAL_OUT_L("==============");
                    // SERIAL_OUT_L("Current:");
                    // SERIAL_OUT("X=");
                    // SERIAL_OUT(_started_l_l.x);
                    // SERIAL_OUT("\tY=");
                    // SERIAL_OUT(_started_l_l.y);
                    // SERIAL_OUT("\tZ=");
                    // SERIAL_OUT(_started_l_l.z);
                    // SERIAL_OUT("\tA=");
                    // SERIAL_OUT(_started_l_l.a);
                    // SERIAL_OUT("\tB=");
                    // SERIAL_OUT(_started_l_l.b);
                    // SERIAL_OUT("\tG=");
                    // SERIAL_OUT_L(_started_l_l.g);

                    // SERIAL_OUT_L("New:");
                    // SERIAL_OUT("X=");
                    // SERIAL_OUT(_executable_leg_left->pos->x);
                    // SERIAL_OUT("\tY=");
                    // SERIAL_OUT(_executable_leg_left->pos->y);
                    // SERIAL_OUT("\tZ=");
                    // SERIAL_OUT(_executable_leg_left->pos->z);
                    // SERIAL_OUT("\tA=");
                    // SERIAL_OUT(_executable_leg_left->pos->a);
                    // SERIAL_OUT("\tB=");
                    // SERIAL_OUT(_executable_leg_left->pos->b);
                    // SERIAL_OUT("\tG=");
                    // SERIAL_OUT_L(_executable_leg_left->pos->g);

                    // SERIAL_OUT("Start time:\t");
                    // SERIAL_OUT_L(_l_l_exec_start_time);
                    // SERIAL_OUT("Execution time:\t");
                    // SERIAL_OUT_L(*(_executable_leg_left->exec_time));
                    // SERIAL_END;
                    func_leg_left();
                    _l_l_last_calc_time = c_time;
                    _update = true;
                }
            }
            else
            {
                if (c_time - _l_l_last_calc_time >= UPDATE_PERIOD)
                {
                    func_leg_left();
                    _l_l_last_calc_time = c_time;
                    _update = true;
                }
            }

            osDelay(1);
            c_time = millis();
            // RIGHT LEG calc

            if (!_executable_leg_right) // if is nullptr
            {
                osMutexWait(_l_r_timeline_mutex, portMAX_DELAY);
                bool empt = _leg_right_timeline.empty();
                osMutexRelease(_l_r_timeline_mutex);
                if (empt == false)
                {
                    osMutexWait(_l_r_exec_mutex, portMAX_DELAY);
                    osMutexWait(_l_r_timeline_mutex, portMAX_DELAY);
                    _executable_leg_right = _leg_right_timeline.front();
                    osMutexRelease(_l_r_exec_mutex);
                    _leg_right_timeline.pop();
                    osMutexRelease(_l_r_timeline_mutex);
                    _current_leg_right->lock_time_mutex();
                    *(_current_leg_right->exec_time) = 0;
                    _current_leg_right->unlock_time_mutex();
                    _l_r_exec_start_time = c_time;
                    _started_l_r = *(_current_leg_right->pos);
                    func_leg_right();
                    _l_r_last_calc_time = c_time;
                    _update = true;
                }
            }
            else
            {
                if (c_time - _l_r_last_calc_time > UPDATE_PERIOD)
                {
                    func_leg_right();
                    _update = true;
                    _l_r_last_calc_time = c_time;
                }
            }

            osDelay(1);
            c_time = millis();
            // hand left

            if (!_executable_hand_left) // if is nullptr
            {
                osMutexWait(_h_l_timeline_mutex, portMAX_DELAY);
                bool empt = _hand_left_timeline.empty();
                osMutexRelease(_h_l_timeline_mutex);
                if (empt == false)
                {
                    osMutexWait(_h_l_exec_mutex, portMAX_DELAY);
                    osMutexWait(_h_l_timeline_mutex, portMAX_DELAY);
                    _executable_hand_left = _hand_left_timeline.front();
                    osMutexRelease(_h_l_exec_mutex);
                    _hand_left_timeline.pop();
                    osMutexRelease(_h_l_timeline_mutex);

                    _current_hand_left->lock_time_mutex();
                    *(_current_hand_left->exec_time) = 0;
                    _current_hand_left->unlock_time_mutex();
                    _h_l_exec_start_time = c_time;
                    _started_h_l = *(_current_hand_left->pos);

                    func_left_hand();
                    _h_l_last_calc_time = c_time;
                    _update = true;
                }
            }
            else
            {
                if (c_time - _h_l_last_calc_time > UPDATE_PERIOD)
                {
                    func_left_hand();
                    _update = true;
                    _h_l_last_calc_time = c_time;
                }
            }

            osDelay(1);
            c_time = millis();
            // hand right

            if (_executable_hand_right == NULL) // if is nullptr
            {
                osMutexWait(_h_r_timeline_mutex, portMAX_DELAY);
                bool empt = _hand_right_timeline.empty();
                osMutexRelease(_h_r_timeline_mutex);
                osDelay(1);
                if (empt == false)
                {
                    osMutexWait(_h_r_exec_mutex, portMAX_DELAY);
                    osMutexWait(_h_r_timeline_mutex, portMAX_DELAY);
                    _executable_hand_right = _hand_right_timeline.front();
                    osMutexRelease(_h_r_exec_mutex);
                    _hand_right_timeline.pop();
                    osMutexRelease(_h_r_timeline_mutex);

                    _current_hand_right->lock_time_mutex();
                    *(_current_hand_right->exec_time) = 0;
                    _current_hand_right->unlock_time_mutex();
                    _h_r_exec_start_time = c_time;
                    _started_h_r = *(_current_hand_left->pos);

                    func_right_hand();
                    _h_r_last_calc_time = c_time;
                    _update = true;
                }
            }
            else
            {
                if (c_time - _h_r_last_calc_time > UPDATE_PERIOD)
                {
                    func_right_hand();
                    _update = true;
                    _h_r_last_calc_time = c_time;
                }
            }

            if (_update)
            {
                dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::velocity_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);
                dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
            }

            osDelay(2);
        }
    }

    osThreadId _traj_thread;

    void trajectory_register_rtos()
    {
        osMutexDef(_l_l_timeline_mutex);
        _l_l_timeline_mutex = osMutexCreate(osMutex(_l_l_timeline_mutex));
        osMutexDef(_l_r_timeline_mutex);
        _l_r_timeline_mutex = osMutexCreate(osMutex(_l_r_timeline_mutex));
        osMutexDef(_h_l_timeline_mutex);
        _h_l_timeline_mutex = osMutexCreate(osMutex(_h_l_timeline_mutex));
        osMutexDef(_h_r_timeline_mutex);
        _h_r_timeline_mutex = osMutexCreate(osMutex(_h_r_timeline_mutex));

        osMutexDef(_l_l_exec_mutex);
        _l_l_exec_mutex = osMutexCreate(osMutex(_l_l_exec_mutex));
        osMutexDef(_l_r_exec_mutex);
        _l_r_exec_mutex = osMutexCreate(osMutex(_l_r_exec_mutex));
        osMutexDef(_h_l_exec_mutex);
        _h_l_exec_mutex = osMutexCreate(osMutex(_h_l_exec_mutex));
        osMutexDef(_h_r_exec_mutex);
        _h_r_exec_mutex = osMutexCreate(osMutex(_h_r_exec_mutex));

        _current_leg_left = std::make_shared<GoalPointerLeg_t>(std::make_shared<kinematics::pos_t>(), std::make_shared<uint32_t>());
        _current_leg_right = std::make_shared<GoalPointerLeg_t>(std::make_shared<kinematics::pos_t>(), std::make_shared<uint32_t>());
        _current_hand_left = std::make_shared<GoalPointerHand_t>(std::make_shared<kinematics::pos_cylindrical_t>(), std::make_shared<uint32_t>());
        _current_hand_right = std::make_shared<GoalPointerHand_t>(std::make_shared<kinematics::pos_cylindrical_t>(), std::make_shared<uint32_t>());

        osThreadDef(_traj_thread, thread_func, osPriority::osPriorityHigh, 1, 4096);
        _traj_thread = osThreadCreate(osThread(_traj_thread), NULL);
        if (!_traj_thread)
            SERIAL_OUT_L_THRSAFE("Not started!!!");
    }

    bool executed_l_l()
    {
        osMutexWait(_l_l_exec_mutex, portMAX_DELAY);
        auto ex = _executable_leg_left;
        osMutexRelease(_l_l_exec_mutex);
        osMutexWait(_l_l_timeline_mutex, portMAX_DELAY);
        bool emp = _leg_left_timeline.empty();
        osMutexRelease(_l_l_timeline_mutex);
        osDelay(1);
        return ex || !emp;
    }

    bool executed_l_r()
    {
        osMutexWait(_l_r_exec_mutex, portMAX_DELAY);
        auto ex = _executable_leg_right;
        osMutexRelease(_l_r_exec_mutex);
        osMutexWait(_l_r_timeline_mutex, portMAX_DELAY);
        bool emp = _leg_right_timeline.empty();
        osMutexRelease(_l_r_timeline_mutex);
        osDelay(1);
        return ex || !emp;
    }

    bool executed_h_l()
    {
        osMutexWait(_h_l_exec_mutex, portMAX_DELAY);
        auto ex = _executable_hand_left;
        osMutexRelease(_h_l_exec_mutex);
        osMutexWait(_h_l_timeline_mutex, portMAX_DELAY);
        bool emp = _hand_left_timeline.empty();
        osMutexRelease(_h_l_timeline_mutex);
        osDelay(1);
        return ex || !emp;
    }

    bool executed_h_r()
    {
        osMutexWait(_h_r_exec_mutex, portMAX_DELAY);
        auto ex = _executable_hand_right;
        osMutexRelease(_h_r_exec_mutex);
        osMutexWait(_h_r_timeline_mutex, portMAX_DELAY);
        bool emp = _hand_right_timeline.empty();
        osMutexRelease(_h_r_timeline_mutex);
        osDelay(1);
        return ex || !emp;
    }

    GoalPointerLeg_t &add_goal_point_leg_left(GoalPointerLeg_t &goal)
    {
        osMutexWait(_l_l_timeline_mutex, portMAX_DELAY);
        _leg_left_timeline.push(&goal);
        osMutexRelease(_l_l_timeline_mutex);
        osDelay(1);
        return goal;
    }

    GoalPointerLeg_t &add_goal_point_leg_right(GoalPointerLeg_t &goal)
    {
        osMutexWait(_l_r_timeline_mutex, portMAX_DELAY);
        _leg_right_timeline.push(&goal);
        osMutexRelease(_l_r_timeline_mutex);
        osDelay(1);
        return goal;
    }

    GoalPointerHand_t &add_goal_point_hand_left(GoalPointerHand_t &goal)
    {
        osMutexWait(_h_l_timeline_mutex, portMAX_DELAY);
        _hand_left_timeline.push(&goal);
        osMutexRelease(_h_l_timeline_mutex);
        osDelay(1);
        return goal;
    }

    GoalPointerHand_t &add_goal_point_hand_right(GoalPointerHand_t &goal)
    {
        osMutexWait(_h_r_timeline_mutex, portMAX_DELAY);
        _hand_right_timeline.push(&goal);
        osMutexRelease(_h_r_timeline_mutex);
        osDelay(1);
        return goal;
    }

    void clear_leg_left_timeline()
    {
        osMutexWait(_l_l_timeline_mutex, portMAX_DELAY);
        _leg_left_timeline = TrajectoryLegsTimeline_t();
        osMutexRelease(_l_l_timeline_mutex);
        osDelay(1);
    }
    void clear_leg_right_timeline()
    {
        osMutexWait(_l_r_timeline_mutex, portMAX_DELAY);
        _leg_right_timeline = TrajectoryLegsTimeline_t();
        osMutexRelease(_l_r_timeline_mutex);
        osDelay(1);
    }
    void clear_right_left_timeline()
    {
        osMutexWait(_h_l_timeline_mutex, portMAX_DELAY);
        _hand_left_timeline = TrajectoryHandsTimeline_t();
        osMutexRelease(_h_l_timeline_mutex);
        osDelay(1);
    }
    void clear_right_right_timeline()
    {
        osMutexWait(_h_r_timeline_mutex, portMAX_DELAY);
        _hand_right_timeline = TrajectoryHandsTimeline_t();
        osMutexRelease(_h_r_timeline_mutex);
        osDelay(1);
    }
    void clear_all_timelines()
    {
        clear_leg_left_timeline();
        clear_leg_left_timeline();
        clear_right_left_timeline();
        clear_right_right_timeline();
    }

    void pause_l_l()
    {
        _pause_l_l = true;
    }
    void pause_l_r()
    {
        _pause_l_r = true;
    }
    void pause_h_l()
    {
        _pause_h_l = true;
    }
    void pause_h_r()
    {
        _pause_h_r = true;
    }
    void pause_all()
    {
        _pause_l_l = true;
        _pause_l_r = true;
        _pause_h_l = true;
        _pause_h_r = true;
    }

    void resume_l_l()
    {
        _pause_l_l = false;
    }
    void resume_l_r()
    {
        _pause_l_r = false;
    }
    void resume_h_l()
    {
        _pause_h_l = false;
    }
    void resume_h_r()
    {
        _pause_h_r = false;
    }
    void resume_all()
    {
        _pause_l_l = false;
        _pause_l_r = false;
        _pause_h_l = false;
        _pause_h_r = false;
    }

    void next_l_l()
    {
    }
    void next_l_r()
    {
    }
    void next_h_l()
    {
    }
    void next_h_r()
    {
    }
    void next_all()
    {
    }

    GoalPointerLeg_t *executable_l_l()
    {
        osMutexWait(_l_l_exec_mutex, portMAX_DELAY);
        auto r = _executable_leg_left;
        osMutexRelease(_l_l_exec_mutex);
        osDelay(1);
        return r;
    }
    GoalPointerLeg_t *executable_l_r()
    {
        osMutexWait(_l_r_exec_mutex, portMAX_DELAY);
        auto r = _executable_leg_right;
        osMutexRelease(_l_r_exec_mutex);
        osDelay(1);
        return r;
    }
    GoalPointerHand_t *executable_h_l()
    {
        osMutexWait(_h_l_exec_mutex, portMAX_DELAY);
        auto r = _executable_hand_left;
        osMutexRelease(_h_l_exec_mutex);
        osDelay(1);
        return r;
    }
    GoalPointerHand_t *executable_h_r()
    {
        osMutexWait(_h_r_exec_mutex, portMAX_DELAY);
        auto r = _executable_hand_right;
        osMutexRelease(_h_r_exec_mutex);
        osDelay(1);
        return r;
    }

    GoalPointerLeg_t &current_l_l()
    {
        return *_current_leg_left;
    }
    GoalPointerLeg_t &current_l_r()
    {
        return *_current_leg_right;
    }
    GoalPointerHand_t &current_h_l()
    {
        return *_current_hand_left;
    }
    GoalPointerHand_t &current_h_r()
    {
        return *_current_hand_right;
    }
}