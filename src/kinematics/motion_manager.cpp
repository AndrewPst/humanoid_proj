#include "motion_manager.h"

#include "../../motion_setup.h"
#include "../../construction.h"

#include "../dynamics/trajectory.h"
#include <array>

namespace motion
{
    // namespace ForwardMoving
    // {
    //     const uint8_t ACTIONS_COUNT{7};

    //     static const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_l_time_line{
    //         {{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + DEFAULT_LEG_LEDGE_Y + LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
    //         {{DEFAULT_LEG_X + FORWARD_MOVING_STEP_BACKWARD_SIZE, -DEFAULT_LEG_LEDGE_Y + DEFAULT_LEG_LEDGE_Y + LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
    //         {{DEFAULT_LEG_X + FORWARD_MOVING_STEP_BACKWARD_SIZE, -DEFAULT_LEG_LEDGE_Y + DEFAULT_LEG_LEDGE_Y + LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    //         {{DEFAULT_LEG_X + FORWARD_MOVING_STEP_BACKWARD_SIZE + FORWARD_MOVING_STEP_FORWARD_SIZE, -DEFAULT_LEG_LEDGE_Y - DEFAULT_LEG_LEDGE_Y - LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_3},
    //         {{DEFAULT_LEG_X + FORWARD_MOVING_STEP_BACKWARD_SIZE + FORWARD_MOVING_STEP_FORWARD_SIZE, -DEFAULT_LEG_LEDGE_Y - DEFAULT_LEG_LEDGE_Y - LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_START, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_4},
    //         {{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + LEGS_DISTANCE_Y, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_END, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_5},
    //         {{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_6},
    //     };

    //     static const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_r_time_line{
    //         {{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + DEFAULT_LEG_LEDGE_Y + LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_START, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
    //         {{DEFAULT_LEG_X - FORWARD_MOVING_STEP_FORWARD_SIZE, DEFAULT_LEG_LEDGE_Y + DEFAULT_LEG_LEDGE_Y + LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_END, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
    //         {{DEFAULT_LEG_X - FORWARD_MOVING_STEP_FORWARD_SIZE, DEFAULT_LEG_LEDGE_Y + DEFAULT_LEG_LEDGE_Y + LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    //         {{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - DEFAULT_LEG_LEDGE_Y - LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_3},
    //         {{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - DEFAULT_LEG_LEDGE_Y - LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_4},
    //         {{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - DEFAULT_LEG_LEDGE_Y - LEGS_DISTANCE_Y / 2.0, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_5},
    //         {{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_6},
    //     };

    //     std::array<std::shared_ptr<trajectory::GoalPointerLeg_t>, ACTIONS_COUNT> _l_l_timeline;
    //     std::array<std::shared_ptr<trajectory::GoalPointerLeg_t>, ACTIONS_COUNT> _l_r_timeline;

    //     void gen_trajectory()
    //     {
    //     }

    //     void pre_init()
    //     {
    //         for (uint8_t i{0}; i < ACTIONS_COUNT; i++)
    //         {
    //             _l_l_timeline[i] = std::make_shared<trajectory::GoalPointerLeg_t>();
    //             _l_r_timeline[i] = std::make_shared<trajectory::GoalPointerLeg_t>();
    //         }
    //     }

    //     void on_start()
    //     {
    //         gen_trajectory();
    //     }

    //     MotionStatus execute()
    //     {
    //     }

    //     void on_end()
    //     {
    //     }

    // }
}