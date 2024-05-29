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

    template <typename _coordT>
    union Pos_arr_t
    {
        _coordT pos{0};
        std::array<double, sizeof(_coordT) / sizeof(double)> arr;
    };

    template <typename _coordT>
    Pos_arr_t<_coordT> make_pos_arr(const _coordT &_pos)
    {
        Pos_arr_t<_coordT> ret;
        ret.pos = _pos;
        return ret;
    }

    using Pos_dec_arr_t = Pos_arr_t<kinematics::pos_dec_t>;
    using Pos_cyl_arr_t = Pos_arr_t<kinematics::pos_cylindrical_t>;
    using Pos_spher_arr_t = Pos_arr_t<kinematics::pos_spherical_t>;

    template <typename _coordT>
    class MovingArg_t
    {
    public:
        _coordT _goal_pos;

        uint32_t _exec_time{0};

        double _acc_time_share_percent{0};
        double _brake_time_share_percent{0};

        bool _mirror{false};
        bool _linear_interpolation{true};

        MovingArg_t()
        {
        }

        MovingArg_t(const _coordT &pos, uint32_t exec_time,
                    double acc, double brake, bool mirror = false,
                    bool lin_interpolation = true) : _goal_pos(pos),
                                                     _exec_time(exec_time),
                                                     _acc_time_share_percent(acc),
                                                     _brake_time_share_percent(brake),
                                                     _mirror(mirror),
                                                     _linear_interpolation(lin_interpolation)
        {
        }
    };

    template <typename _coordT>
    class MovingArg_thr_save_t
    {
    private:
        MovingArg_t<_coordT> _args;

        osMutexDef(_pos_mutex);
        osMutexId(_pos_mutex_id);

        osMutexDef(_time_mutex);
        osMutexId(_time_mutex_id);

        osMutexDef(_acc_mutex);
        osMutexId(_acc_mutex_id);

        osMutexDef(_brake_mutex);
        osMutexId(_brake_mutex_id);

        void init_mutexes()
        {
            // if (!_pos_mutex_id)
            _pos_mutex_id = osMutexCreate(osMutex(_pos_mutex));
            // if (!_time_mutex_id)
            _time_mutex_id = osMutexCreate(osMutex(_time_mutex));
            // if (!_acc_mutex_id)
            _acc_mutex_id = osMutexCreate(osMutex(_acc_mutex));
            // if (!_brake_mutex_id)
            _brake_mutex_id = osMutexCreate(osMutex(_brake_mutex));
        }

        friend void trajectory_register_rtos();

    public:
        MovingArg_thr_save_t()
        {
            init_mutexes();
        }

        _coordT &pos_thr_safe()
        {
            osMutexWait(_pos_mutex_id, 0);
            return _args._goal_pos;
        }
        void free_pos_mutex()
        {
            osMutexRelease(_pos_mutex_id);
        }

        double &exec_time_thr_safe()
        {
            osMutexWait(_time_mutex_id, 0);
            return _args._exec_time;
        }
        void free_exec_time_mutex()
        {
            osMutexRelease(_time_mutex_id);
        }

        double &acc_thr_safe()
        {
            osMutexWait(_acc_mutex_id, 0);
            return _args._acc_time_share_percent;
        }
        void free_acc_mutex()
        {
            osMutexRelease(_acc_mutex_id);
        }

        double &brake_thr_safe()
        {
            osMutexWait(_brake_mutex_id, 0);
            return _args._brake_time_share_percent;
        }
        void free_brake_mutex()
        {
            osMutexRelease(_brake_mutex_id);
        }
        // TODO: create Mutate func
    };

    enum ExecutionStatus : uint8_t
    {
        EXEC_FINISHED_SUCC = 0,
        EXEC_IN_PROCESS,
        EXEC_SUSPENDED,
        EXEC_ERROR_STOPPED
    };

    enum ExecutionStep : uint8_t
    {
        EXEC_STEP_FINISHED = 0,
        EXEC_STEP_ACCELERATION,
        EXEC_STEP_UNIFORM_MOVEMENT,
        EXEC_STEP_BRAKE,
        EXEC_STEP_ERROR,
    };

    using MovingArg_leg_t = MovingArg_t<Pos_dec_arr_t>;
    using MovingArg_hand_t = MovingArg_t<Pos_cyl_arr_t>;

    using MovingArg_leg_thr_save_t = MovingArg_thr_save_t<Pos_dec_arr_t>;
    using MovingArg_hand_thr_save_t = MovingArg_thr_save_t<Pos_cyl_arr_t>;

    uint8_t set_new_goal_pos_l_l(const MovingArg_leg_t &pos);
    uint8_t set_new_goal_pos_l_r(const MovingArg_leg_t &pos);
    uint8_t set_new_goal_pos_h_l(const MovingArg_hand_t &pos);
    uint8_t set_new_goal_pos_h_r(const MovingArg_hand_t &pos);

    const MovingArg_leg_t &get_goal_pos_l_l();
    const MovingArg_leg_t &get_goal_pos_l_r();
    const MovingArg_hand_t &get_goal_pos_h_l();
    const MovingArg_hand_t &get_goal_pos_h_r();

    const std::shared_ptr<MovingArg_leg_thr_save_t> &get_current_pos_l_l();
    const std::shared_ptr<MovingArg_leg_thr_save_t> &get_current_pos_l_r();
    const std::shared_ptr<MovingArg_hand_thr_save_t> &get_current_pos_h_l();
    const std::shared_ptr<MovingArg_hand_thr_save_t> &get_current_pos_h_r();

    ExecutionStatus get_moving_status_l_l();
    ExecutionStatus get_moving_status_l_r();
    ExecutionStatus get_moving_status_h_l();
    ExecutionStatus get_moving_status_h_r();

    void move_head(double a, double b);

    void wait_all_executed();

    // set startup positions of limb for correctly moving to first foal position
    void set_startup_position(const kinematics::pos_dec_t &_l_l_pos,
                              const kinematics::pos_dec_t &_l_r_pos,
                              const kinematics::pos_cylindrical_t &_h_l_pos,
                              const kinematics::pos_cylindrical_t &_h_r_pos);

    void trajectory_register_rtos();

    const std::array<double, SERVO_COUNT> &get_last_rads_buff();
    const std::array<double, SERVO_COUNT> &get_ik_rads_buff();
}