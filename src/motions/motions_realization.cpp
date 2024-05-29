#include "motions.h"
#include "../../motion_setup.h"

namespace motions
{
#pragma region Forward movement

    void ForwardMotion::start()
    {
        iter_counter = 0;
    }

    MotionStatus ForwardMotion::exec()
    {
        if (trajectory::get_moving_status_l_l() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC)
        {
            trajectory::set_new_goal_pos_l_l(_l_l_time_line[_l_l_id]);
            _l_l_id++;
            if (_l_l_id >= ACTIONS_COUNT)
                _l_l_id = 0;
            iter_counter++;
        }
        if (trajectory::get_moving_status_l_r() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC)
        {
            trajectory::set_new_goal_pos_l_r(_l_r_time_line[_l_r_id]);
            _l_r_id++;
            if (_l_r_id >= ACTIONS_COUNT)
                _l_r_id = 0;
        }
        if (trajectory::get_moving_status_h_l() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC)
        {
            trajectory::set_new_goal_pos_h_l(_h_l_time_line[_h_l_id]);
            _h_l_id++;
            if (_h_l_id >= ACTIONS_COUNT)
                _h_l_id = 0;
        }
        if (trajectory::get_moving_status_h_r() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC)
        {
            trajectory::set_new_goal_pos_h_r(_h_r_time_line[_h_r_id]);
            _h_r_id++;
            if (_h_r_id >= ACTIONS_COUNT)
                _h_r_id = 0;
        }
        if (iter_counter > 10)
        {
            iter_counter = 0;
            trajectory::wait_all_executed();
        }
        return MotionStatus::M_STATUS_IN_EXECUTION;
    }

    void ForwardMotion::end()
    {
    }

    const std::array<trajectory::MovingArg_leg_t, ACTIONS_COUNT> ForwardMotion::_l_l_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}), 1000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}), 1000, 0.25, 0.25},
    }};

    const std::array<trajectory::MovingArg_leg_t, ACTIONS_COUNT> ForwardMotion::_l_r_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}), 1000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}), 1000, 0.25, 0.25},
    }};

    const std::array<trajectory::MovingArg_hand_t, ACTIONS_COUNT> ForwardMotion::_h_l_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({0, 180, 60}), 500, 0.5, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_l_pos), 500, 0.0, 0.5},
    }};

    const std::array<trajectory::MovingArg_hand_t, ACTIONS_COUNT> ForwardMotion::_h_r_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({0, 180, 60}), 500, 0.5, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_r_pos), 500, 0.0, 0.5},
    }};

#pragma endregion

#pragma region Falling back movement

    void FallingBackMotion::start()
    {
        iter_counter = 0;
        SERIAL_OUT_L_THRSAFE("Back!");
    }

    MotionStatus FallingBackMotion::exec()
    {
        if (iter_counter == 0)
        {

            trajectory::set_new_goal_pos_l_l(_l_l_time_line[0]);
            trajectory::set_new_goal_pos_l_r(_l_r_time_line[0]);
            trajectory::set_new_goal_pos_h_l(_h_l_time_line[0]);
            trajectory::set_new_goal_pos_h_r(_h_r_time_line[0]);
            iter_counter++;
        }
        else
        {
            if (trajectory::get_moving_status_l_l() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC &&
                trajectory::get_moving_status_l_r() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC &&
                trajectory::get_moving_status_h_l() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC &&
                trajectory::get_moving_status_h_r() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC)
            {
                if (iter_counter == FALLING_BACK_ACTIONS_COUNT)
                    return MotionStatus::M_STATUS_ENDED;

                trajectory::set_new_goal_pos_l_l(_l_l_time_line[iter_counter]);
                trajectory::set_new_goal_pos_l_r(_l_r_time_line[iter_counter]);
                trajectory::set_new_goal_pos_h_l(_h_l_time_line[iter_counter]);
                trajectory::set_new_goal_pos_h_r(_h_r_time_line[iter_counter]);
                iter_counter++;
            }
        }
        return MotionStatus::M_STATUS_IN_EXECUTION;
    }

    void FallingBackMotion::end()
    {
    }

    const std::array<trajectory::MovingArg_leg_t, FALLING_BACK_ACTIONS_COUNT> FallingBackMotion::_l_l_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, -0, 0, 0}), 2000, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, -0, 0, 0}), 2000, 0.0, 0.0},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-100, 0, DEFAULT_LEG_Z - 50, -0, 0, -90 * DEG_TO_RAD}), 2000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-100, 0, DEFAULT_LEG_Z - 110, -0, 0, -90 * DEG_TO_RAD}), 2000, 0.25, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -0, 0, -85 * DEG_TO_RAD}), 2000, 0.0, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -30, 0, -85 * DEG_TO_RAD}), 300, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -30, 0, -85 * DEG_TO_RAD}), 2000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -30, 0, -85 * DEG_TO_RAD}), 2000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -0, 0, -85 * DEG_TO_RAD}), 300, 0.25, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 30, 0, -85 * DEG_TO_RAD}), 300, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 30, 0, -85 * DEG_TO_RAD}), 2000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 30, 0, -85 * DEG_TO_RAD}), 2000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 0, 0, -85 * DEG_TO_RAD}), 300, 0.25, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -0, 0, -85 * DEG_TO_RAD}), 10000, 0.25, 0.25},

        // {trajectory::make_pos_arr<kinematics::pos_dec_t>({-75, 0, DEFAULT_LEG_Z - 40, -0, 0, -45 * DEG_TO_RAD}), 1500, 0.25, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_l_pos), 2000, 0.0, 0.0},
    }};

    const std::array<trajectory::MovingArg_leg_t, FALLING_BACK_ACTIONS_COUNT> FallingBackMotion::_l_r_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, 0, 0, 0}), 2000, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, 0, 0, 0}), 2000, 0.0, 0.0},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-100, 0, DEFAULT_LEG_Z - 50, 0, 0, -90 * DEG_TO_RAD}), 2000, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-100, 0, DEFAULT_LEG_Z - 110, 0, 0, -90 * DEG_TO_RAD}), 2000, 0.25, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 0, 0, -85 * DEG_TO_RAD}), 2000, 0.00, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 30, 0, -85 * DEG_TO_RAD}), 300, 0.00, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 30, 0, -85 * DEG_TO_RAD}), 2000, 0.00, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 30, 0, -85 * DEG_TO_RAD}), 2000, 0.00, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 0, 0, -85 * DEG_TO_RAD}), 300, 0.00, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -30, 0, -85 * DEG_TO_RAD}), 300, 0.00, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -30, 0, -85 * DEG_TO_RAD}), 2000, 0.00, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, -30, 0, -85 * DEG_TO_RAD}), 2000, 0.00, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 0, 0, -85 * DEG_TO_RAD}), 300, 0.00, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>({-95, 0, DEFAULT_LEG_Z - 134, 0, 0, -85 * DEG_TO_RAD}), 10000, 0.25, 0.25},

        // {trajectory::make_pos_arr<kinematics::pos_dec_t>({-75, 0, DEFAULT_LEG_Z - 40, -0, 0, -45 * DEG_TO_RAD}), 1500, 0.25, 0.25},

        {trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_r_pos), 2000, 0.0, 0.0},
    }};

    const std::array<trajectory::MovingArg_hand_t, FALLING_BACK_ACTIONS_COUNT> FallingBackMotion::_h_l_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-60 * DEG_TO_RAD, 195, 40}), 2000, 0.0, 0.0, true, false},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-60 * DEG_TO_RAD, 165, 40}), 2000, 0.0, 0.5, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-80 * DEG_TO_RAD, 195, 40}), 2000, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, true, true},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 165, 22}), 300, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({0 * DEG_TO_RAD, DEFAULT_HAND_R, DEFAULT_HAND_Z}), 2000, 0.25, 0.25, false, false},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 165, 22}), 2000, 0.25, 0.25, false, false},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 300, 0.25, 0.25, false, true},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 300, 0.25, 0.25, false, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, false, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, false, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 300, 0.25, 0.25, false, true},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 10000, 0.25, 0.25, false, false},

        // {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-110 * DEG_TO_RAD, 165, 40}), 1500, 0.0, 0.0, true, false},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_l_pos), 2000, 0.0, 0.0, false, false}, // на поднятие пауза
    }};

    const std::array<trajectory::MovingArg_hand_t, FALLING_BACK_ACTIONS_COUNT> FallingBackMotion::_h_r_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-60 * DEG_TO_RAD, 195, 40}), 2000, 0.0, 0.0, true, false},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-60 * DEG_TO_RAD, 165, 40}), 2000, 0.0, 0.5, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-80 * DEG_TO_RAD, 195, 40}), 2000, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, true, true},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 300, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 2000, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 300, 0.25, 0.25, true, true},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 165, 22}), 300, 0.25, 0.25, true, true},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-0 * DEG_TO_RAD, DEFAULT_HAND_R, DEFAULT_HAND_Z}), 2000, 0.25, 0.25, false, false},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 165, 22}), 2000, 0.25, 0.25, false, false},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 300, 0.25, 0.25, false, true},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-100 * DEG_TO_RAD, 195, 22}), 10000, 0.25, 0.25, false, false},

        // {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-110 * DEG_TO_RAD, 165, 40}), 1500, 0.0, 0.0, true, false},

        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_r_pos), 2000, 0.0, 0.0, false, false}, // на поднятие пауза
    }};

    // const std::array<trajectory::MovingArg_leg_t, FALLING_BACK_ACTIONS_COUNT> FallingBackMotion::_l_r_time_line = {{
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 85, 0, DEFAULT_LEG_Z - 50, 0, 0, -10 * DEG_TO_RAD}), 2000, 0.0, 0.0},
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 85, 0, DEFAULT_LEG_Z - 80, 0, 0, -15 * DEG_TO_RAD}), 200, 0.0, 0.4},
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 85, 0, DEFAULT_LEG_Z - 80, 0, 0, -10 * DEG_TO_RAD}), 2000, 0.0, 0.0},
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 70, 0, DEFAULT_LEG_Z - 65, 0, 0, -15 * DEG_TO_RAD}), 200, 0.0, 0.0},
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 50, 0, DEFAULT_LEG_Z - 35, 0, 0, -8 * DEG_TO_RAD}), 10, 0.0, 0.0},
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X - 30, 0, DEFAULT_LEG_Z - 30, 0, 0, -5 * DEG_TO_RAD}), 200, 0.0, 0.5},
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X - 30, 0, DEFAULT_LEG_Z - 30, 0, 0, 10 * DEG_TO_RAD}), 500, 0.5, 0.5},
    //     {trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_r_pos), 2000, 0.0, 0.0},
    // }};

    // const std::array<trajectory::MovingArg_hand_t, FALLING_BACK_ACTIONS_COUNT> FallingBackMotion::_h_l_time_line = {{
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-110 * DEG_TO_RAD, 195, 40}), 2000, 0.0, 0.0, true, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-140 * DEG_TO_RAD, 195, 40}), 200, 0.0, 0.0, true, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({0 * DEG_TO_RAD, 155, 40}), 2000, 0.0, 0.0, false, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-20 * DEG_TO_RAD, 195, 22}), 190, 0.0, 0.0, false, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({0 * DEG_TO_RAD, 195, 22}), 100, 0.0, 0.0, false, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({30 * DEG_TO_RAD, 195, 22}), 200, 0.0, 0.0, false, true},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({60 * DEG_TO_RAD, 195, 22}), 500, 0.0, 0.0, false, true},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_l_pos), 2000, 0.0, 0.0, false, false}, // на поднятие пауза
    // }};

    // const std::array<trajectory::MovingArg_hand_t, FALLING_BACK_ACTIONS_COUNT> FallingBackMotion::_h_r_time_line = {{
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-110 * DEG_TO_RAD, 195, 40}), 2000, 0.0, 0.0, true, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-140 * DEG_TO_RAD, 195, 40}), 200, 0.0, 0.0, true, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({0 * DEG_TO_RAD, 155, 40}), 2000, 0.0, 0.0, false, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({-20 * DEG_TO_RAD, 195, 22}), 190, 0.0, 0.0, false, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({0 * DEG_TO_RAD, 195, 22}), 100, 0.0, 0.0, false, false},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({30 * DEG_TO_RAD, 195, 22}), 200, 0.0, 0.0, false, true},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({60 * DEG_TO_RAD, 195, 22}), 500, 0.0, 0.0, false, true},
    //     {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_r_pos), 2000, 0.0, 0.0, false, false}, // на поднятие пауза
    // }};

#pragma endregion

#pragma region Falling front movement

    void FallingForwardMotion::start()
    {
        iter_counter = 0;
        SERIAL_OUT_L_THRSAFE("Front!");
    }

    MotionStatus FallingForwardMotion::exec()
    {
        if (iter_counter == 0)
        {

            trajectory::set_new_goal_pos_l_l(_l_l_time_line[0]);
            trajectory::set_new_goal_pos_l_r(_l_r_time_line[0]);
            trajectory::set_new_goal_pos_h_l(_h_l_time_line[0]);
            trajectory::set_new_goal_pos_h_r(_h_r_time_line[0]);
            iter_counter++;
        }
        else
        {
            if (trajectory::get_moving_status_l_l() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC &&
                trajectory::get_moving_status_l_r() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC &&
                trajectory::get_moving_status_h_l() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC &&
                trajectory::get_moving_status_h_r() == trajectory::ExecutionStatus::EXEC_FINISHED_SUCC)
            {
                if (iter_counter == FALLING_FRONT_ACTIONS_COUNT)
                    return MotionStatus::M_STATUS_ENDED;

                trajectory::set_new_goal_pos_l_l(_l_l_time_line[iter_counter]);
                trajectory::set_new_goal_pos_l_r(_l_r_time_line[iter_counter]);
                trajectory::set_new_goal_pos_h_l(_h_l_time_line[iter_counter]);
                trajectory::set_new_goal_pos_h_r(_h_r_time_line[iter_counter]);
                iter_counter++;
            }
        }
        return MotionStatus::M_STATUS_IN_EXECUTION;
    }

    void FallingForwardMotion::end()
    {
    }

    const std::array<trajectory::MovingArg_leg_t, FALLING_FRONT_ACTIONS_COUNT> FallingForwardMotion::_l_l_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 40, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z + 20, 0, 0, -25 * DEG_TO_RAD}), 10, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 40, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z + 20, 0, 0, -25 * DEG_TO_RAD}), 300, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, -0, DEFAULT_LEG_Z, 0, 0, -25 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 125, -0, DEFAULT_LEG_Z - 100, 0, 0, 80 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 125, -DEFAULT_LEG_LEDGE_Y + 20, DEFAULT_LEG_Z - 100, 0, 0, 80 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 125, -0, DEFAULT_LEG_Z - 100, 0, 0, 80 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 120, -0, DEFAULT_LEG_Z - 120, 0, 0, 80 * DEG_TO_RAD}), 300, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 120, -0, DEFAULT_LEG_Z - 100, 0, 0, 40 * DEG_TO_RAD}), 800, 0.0, 0.8},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 60, -0, DEFAULT_LEG_Z - 40, 0, 0, 15 * DEG_TO_RAD}), 1000, 0.0, 0.4},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_l_pos), 800, 0.2, 0.5},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_l_pos), 5000, 0.0, 0.0},
    }};

    const std::array<trajectory::MovingArg_leg_t, FALLING_FRONT_ACTIONS_COUNT> FallingForwardMotion::_l_r_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 40, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z + 20, 0, 0, -25 * DEG_TO_RAD}), 10, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 40, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z + 20, 0, 0, -25 * DEG_TO_RAD}), 300, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 50, DEFAULT_LEG_LEDGE_Y - 20, DEFAULT_LEG_Z, 0, 0, -25 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 50, DEFAULT_LEG_LEDGE_Y - 20, DEFAULT_LEG_Z, 0, 0, -25 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X, 0, DEFAULT_LEG_Z, 0, 0, 0 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 125, 0, DEFAULT_LEG_Z - 100, 0, 0, 80 * DEG_TO_RAD}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 120, 0, DEFAULT_LEG_Z - 120, 0, 0, 80 * DEG_TO_RAD}), 300, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 120, 0, DEFAULT_LEG_Z - 100, 0, 0, 40 * DEG_TO_RAD}), 800, 0.0, 0.8},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>({DEFAULT_LEG_X + 60, 0, DEFAULT_LEG_Z - 40, 0, 0, 15 * DEG_TO_RAD}), 1000, 0.0, 0.4},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_r_pos), 800, 0.2, 0.5},
        {trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_r_pos), 5000, 0.25, 0.25},
    }};

    const std::array<trajectory::MovingArg_hand_t, FALLING_FRONT_ACTIONS_COUNT> FallingForwardMotion::_h_l_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 145, 60}), 1, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 145, 60}), 300, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 60}), 400, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 30}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 0}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 30}), 400, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 25 * DEG_TO_RAD, 195, 30}), 300, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2, 200, 22}), 1, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_l_pos), 1000, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_l_pos), 400, 0.4, 0.4},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_l_pos), 5000, 0.0, 0.0}, // на поднятие пауза
    }};

    const std::array<trajectory::MovingArg_hand_t, FALLING_FRONT_ACTIONS_COUNT> FallingForwardMotion::_h_r_time_line = {{
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 145, 60}), 1, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 145, 60}), 300, 0.0, 0.0},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 0}), 400, 0.25, 0.25},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 30}), 400, 0.0, 0.0}, // думаем
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 60}), 400, 0.0, 0.0}, // думаем
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 20 * DEG_TO_RAD, 195, 30}), 400, 0.0, 0.0}, // думаем
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2 + 25 * DEG_TO_RAD, 165, 30}), 300, 0.0, 0.0}, // думаем
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>({M_PI_2, 200, 22}), 1, 0.0, 0.0},                     // думаем
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_r_pos), 1000, 0.0, 0.0},                   // думаем
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_r_pos), 400, 0.4, 0.4},
        {trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_r_pos), 5000, 0.0, 0.0}, // на поднятие пауза
    }};

#pragma endregion

}