#pragma once

#include "../humanoid/humanoid.h"
#include "../kinematics/kinematics.h"

#include "../../sout.h"

#include <RTOS.h>

#include <inttypes.h>
#include <queue>
#include <array>
#include <memory>

namespace trajectory
{
    template <typename coord_type_T>
    struct GoalPointer_t
    {
        osMutexDef(_pos_mutex);
        osMutexId(_pos_mutex_id);

        osMutexDef(_time_mutex);
        osMutexId(_time_mutex_id);

        std::shared_ptr<coord_type_T> pos;
        std::shared_ptr<uint32_t> exec_time;

        GoalPointer_t() : pos(0), exec_time(0)
        {
            init_mutexes();
        }

        GoalPointer_t(std::shared_ptr<coord_type_T> p, std::shared_ptr<uint32_t> t) : pos(p), exec_time(t)
        {
            init_mutexes();
        }

        void lock_pos_mutex()
        {
            osMutexWait(_pos_mutex_id, 0);
        }
        void unlock_pos_mutex()
        {
            osMutexRelease(_pos_mutex_id);
            osDelay(1);
        }

        void lock_time_mutex()
        {
            osMutexWait(_time_mutex_id, 0);
        }

        void unlock_time_mutex()
        {
            osMutexRelease(_time_mutex_id);
            osDelay(1);
        }

        ~GoalPointer_t()
        {
            if (_pos_mutex_id)
                osMutexDelete(_pos_mutex_id);
            if (_time_mutex_id)
                osMutexDelete(_time_mutex_id);
        }

    private:
        void init_mutexes()
        {
            _pos_mutex_id = osMutexCreate(osMutex(_pos_mutex));
            if (!_pos_mutex_id)
                SERIAL_OUT_L_THRSAFE("Mut create err");
            _time_mutex_id = osMutexCreate(osMutex(_time_mutex));
            if (!_time_mutex_id)
                SERIAL_OUT_L_THRSAFE("Mut create err");
        }
    };

    typedef GoalPointer_t<kinematics::pos_t> GoalPointerLeg_t;
    typedef GoalPointer_t<kinematics::pos_cylindrical_t> GoalPointerHand_t;

    typedef std::queue<GoalPointerLeg_t *> TrajectoryLegsTimeline_t;
    typedef std::queue<GoalPointerHand_t *> TrajectoryHandsTimeline_t;

    void trajectory_register_rtos();

    GoalPointerLeg_t &add_goal_point_leg_left(GoalPointerLeg_t &goal);
    GoalPointerLeg_t &add_goal_point_leg_right(GoalPointerLeg_t &goal);
    GoalPointerHand_t &add_goal_point_hand_left(GoalPointerHand_t &goal);
    GoalPointerHand_t &add_goal_point_hand_right(GoalPointerHand_t &goal);

    void clear_leg_left_timeline();
    void clear_leg_right_timeline();
    void clear_right_left_timeline();
    void clear_right_right_timeline();
    void clear_all_timelines();

    void pause_l_l();
    void pause_l_r();
    void pause_h_l();
    void pause_h_r();
    void pause_all();

    void resume_l_l();
    void resume_l_r();
    void resume_h_l();
    void resume_h_r();
    void resume_all();

    void next_l_l();
    void next_l_r();
    void next_h_l();
    void next_h_r();
    void next_all();

    GoalPointerLeg_t *executable_l_l();
    GoalPointerLeg_t *executable_l_r();
    GoalPointerHand_t *executable_h_l();
    GoalPointerHand_t *executable_h_r();

    GoalPointerLeg_t &current_l_l();
    GoalPointerLeg_t &current_l_r();
    GoalPointerHand_t &current_h_l();
    GoalPointerHand_t &current_h_r();

    const std::array<double, SERVO_COUNT> &get_last_rads_buff();
    const std::array<double, SERVO_COUNT> &get_ik_rads_buff();

    bool executed_l_l();
    bool executed_l_r();
    bool executed_h_l();
    bool executed_h_r();

    // void recalc_l_l();
    // void recalc_l_r();
    // void recalc_h_l();
    // void recalc_h_r();
    // void recalc_all();

}