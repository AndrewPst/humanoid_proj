#include <Arduino.h>
#include <RTOS.h>

#include "motion_setup.h"
#include "src/kinematics/kinematics.h"
#include "sout.h"
#include "src/dynamixel_drivers/dynamixel.h"
#include "src/humanoid/humanoid.h"
#include "src/lidar/lidar.h"
#include "src/camera/camera.h"
#include "src/orientation/orientation.h"
#include "src/dynamics/trajectory.h"

#include "construction.h"
#include "config.h"

#include <inttypes.h>
#include <vector>
#include <array>
#include <utility>

osMutexId _s_mutex{nullptr};
osThreadId thread_id_main{nullptr};

//--------------Default and startup body position-----------
const kinematics::pos_t _default_l_l_pos{
    DEFAULT_LEG_X,
    -DEFAULT_LEG_LEDGE_Y,
    DEFAULT_LEG_Z,
    -DEFAULT_LEG_CONVERGANCE,
    DEFAULT_LEG_COLLAPSE,
    DEFAULT_LEG_CASTER,
};
const kinematics::pos_t _default_l_r_pos{
    DEFAULT_LEG_X,
    DEFAULT_LEG_LEDGE_Y,
    DEFAULT_LEG_Z,
    DEFAULT_LEG_CONVERGANCE,
    DEFAULT_LEG_COLLAPSE,
    DEFAULT_LEG_CASTER,
};
const kinematics::pos_cylindrical_t _default_h_l_pos{
    DEFAULT_HAND_ANGLE,
    DEFAULT_HAND_R,
    DEFAULT_HAND_Z,
};
const kinematics::pos_cylindrical_t _default_h_r_pos{
    DEFAULT_HAND_ANGLE,
    DEFAULT_HAND_R,
    DEFAULT_HAND_Z,
};

const uint8_t ACTIONS_COUNT = 8;

const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_l_time_line_1{
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 20, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 20, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + 20, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + 20, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
};

const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_r_time_line_1{
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - 20, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - 20, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 20, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 20, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
    std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
};
// const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_l_time_line_1{
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z + 8, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z - 5, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z - 5, -DEFAULT_LEG_CONVERGANCE + 4 * DEG_TO_RAD, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE + 4 * DEG_TO_RAD, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE + 4 * DEG_TO_RAD, DEFAULT_LEG_COLLAPSE + 10 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, -DEFAULT_LEG_LEDGE_Y - 15, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE + 2 * DEG_TO_RAD, DEFAULT_LEG_COLLAPSE + 8 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
// };

// const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_r_time_line_1{
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - 10, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE - 10 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 10, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE - 10 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 20, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE - 8 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z + 8, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z - 5, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z - 5, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
// };

// const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_l_time_line_2{
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + 10, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE + 10 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 10, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE + 10 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 20, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE + 8 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z + 8, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - 30, DEFAULT_LEG_Z - 5, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z - 5, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
// };

// const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_r_time_line_2{
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z + 8, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z - 5, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z - 5, DEFAULT_LEG_CONVERGANCE , DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE , DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, DEFAULT_LEG_LEDGE_Y + 30, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE , DEFAULT_LEG_COLLAPSE - 10 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + 8, DEFAULT_LEG_LEDGE_Y + 15, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE , DEFAULT_LEG_COLLAPSE - 8 * DEG_TO_RAD, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
// };

// const std::array<std::pair<kinematics::pos_cylindrical_t, uint32_t>, ACTIONS_COUNT> _base_h_l_time_line{
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{60 * DEG_TO_RAD, 190, 40}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{60 * DEG_TO_RAD, 190, 40}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{0 * DEG_TO_RAD, 190, 40}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{0 * DEG_TO_RAD, 20, 190}, FORWARD_MOVING_OPER_3},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{0 * DEG_TO_RAD, 20, 190}, FORWARD_MOVING_OPER_4},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{0 * DEG_TO_RAD, 0, 0}, FORWARD_MOVING_OPER_5},
// };

// const std::array<std::pair<kinematics::pos_cylindrical_t, uint32_t>, ACTIONS_COUNT> _base_h_r_time_line{
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{60 * DEG_TO_RAD, 20, 190}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{60 * DEG_TO_RAD, 20, 190}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{0 * DEG_TO_RAD, 190, 40}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{60 * DEG_TO_RAD, 190, 40}, FORWARD_MOVING_OPER_3},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{60 * DEG_TO_RAD, 190, 40}, FORWARD_MOVING_OPER_4},
//     std::pair<kinematics::pos_cylindrical_t, uint32_t>{{0 * DEG_TO_RAD, 0, 0}, FORWARD_MOVING_OPER_5},
// }
// ;
// const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_l_time_line{
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y + FORWARD_MOVING_ONE_LEG_Y_INNER, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE + FORWARD_MOVING_ONE_LEG_B, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X - FORWARD_MOVING_STEP_BACKWARD_SIZE, -DEFAULT_LEG_LEDGE_Y + FORWARD_MOVING_ONE_LEG_Y_INNER, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE + FORWARD_MOVING_ONE_LEG_B, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X - FORWARD_MOVING_STEP_BACKWARD_SIZE, -DEFAULT_LEG_LEDGE_Y + FORWARD_MOVING_ONE_LEG_Y_INNER, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X - FORWARD_MOVING_STEP_BACKWARD_SIZE - FORWARD_MOVING_STEP_FORWARD_SIZE, -DEFAULT_LEG_LEDGE_Y - FORWARD_MOVING_ONE_LEG_Y_OUTER, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_3},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X - FORWARD_MOVING_STEP_BACKWARD_SIZE - FORWARD_MOVING_STEP_FORWARD_SIZE, -DEFAULT_LEG_LEDGE_Y - FORWARD_MOVING_ONE_LEG_Y_OUTER, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_START, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_4},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y - FORWARD_MOVING_ONE_LEG_Y_OUTER, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_END, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_5},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, -DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, -DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_6}};

// const std::array<std::pair<kinematics::pos_t, uint32_t>, ACTIONS_COUNT> _base_l_r_time_line{
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y + FORWARD_MOVING_ONE_LEG_Y_OUTER, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_START, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_0},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + FORWARD_MOVING_STEP_FORWARD_SIZE, DEFAULT_LEG_LEDGE_Y + FORWARD_MOVING_ONE_LEG_Y_OUTER, DEFAULT_LEG_Z - FORWARD_MOVING_STEP_HEIGHT_END, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_1},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X + FORWARD_MOVING_STEP_FORWARD_SIZE, DEFAULT_LEG_LEDGE_Y + FORWARD_MOVING_ONE_LEG_Y_OUTER, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_2},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - FORWARD_MOVING_ONE_LEG_Y_INNER, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_3},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - FORWARD_MOVING_ONE_LEG_Y_INNER, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE - FORWARD_MOVING_ONE_LEG_B, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_4},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y - FORWARD_MOVING_ONE_LEG_Y_INNER, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE - FORWARD_MOVING_ONE_LEG_B, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_5},
//     std::pair<kinematics::pos_t, uint32_t>{{DEFAULT_LEG_X, DEFAULT_LEG_LEDGE_Y, DEFAULT_LEG_Z, DEFAULT_LEG_CONVERGANCE, DEFAULT_LEG_COLLAPSE, DEFAULT_LEG_CASTER}, FORWARD_MOVING_OPER_6}};

enum InitResult : uint8_t
{
    INIT_SUCC = 0,
    INIT_LEFT_LEG_ERROR_PING,
    INIT_RIGHT_LEG_ERROR_PING,
    INIT_LEFT_HAND_ERROR_PING,
    INIT_RIGHT_HAND_ERROR_PING,
    INIT_RIGHT_HEAD_ERROR_PING,
    INIT_SETUP_DRIVERS_ERROR,
    INIT_DYNAMIXEL_DRIVER_ERROR,
    INIT_CAMERA_NOT_FOUND_ERROR,
    INIT_ORIENT_ERROR,
};

void main_register_rtos()
{
    osMutexDef(_s_mutex);
    _s_mutex = osMutexCreate(osMutex(_s_mutex));

    osThreadDef(thread_id_main, thread_main, osPriorityNormal, 1, 8192 * 2);
    thread_id_main = osThreadCreate(osThread(thread_id_main), NULL);
}

std::shared_ptr<trajectory::GoalPointerLeg_t> _l_l_pointer;
std::shared_ptr<trajectory::GoalPointerLeg_t> _l_r_pointer;
std::shared_ptr<trajectory::GoalPointerHand_t> _h_l_pointer;
std::shared_ptr<trajectory::GoalPointerHand_t> _h_r_pointer;

void setup()
{
    //  Register tasks
    main_register_rtos();

    SERIAL_BEGIN
    SERIAL_INIT
    SERIAL_END

    humanoid::humanoid_register_rtos();
    // camera::camera_register_rtos();
    // orientation::register_rtos();
    dynamixel::dynamixel_register_rtos();
    trajectory::trajectory_register_rtos();

    _l_l_pointer = std::make_shared<trajectory::GoalPointerLeg_t>(std::make_shared<kinematics::pos_t>(), std::make_shared<uint32_t>());
    _l_r_pointer = std::make_shared<trajectory::GoalPointerLeg_t>(std::make_shared<kinematics::pos_t>(), std::make_shared<uint32_t>());
    _h_l_pointer = std::make_shared<trajectory::GoalPointerHand_t>(std::make_shared<kinematics::pos_cylindrical_t>(), std::make_shared<uint32_t>());
    _h_r_pointer = std::make_shared<trajectory::GoalPointerHand_t>(std::make_shared<kinematics::pos_cylindrical_t>(), std::make_shared<uint32_t>());

    // start kernel
    osKernelStart();
}

InitResult initsystem()
{
    kinematics::set_max_values_legs_joints(humanoid::get_max_angles_of_limb(humanoid::LimbId::LIMB_LEG_LEFT));
    kinematics::set_min_values_legs_joints(humanoid::get_min_angles_of_limb(humanoid::LimbId::LIMB_LEG_LEFT));
    kinematics::set_min_values_hands_joints(humanoid::get_min_angles_of_limb(humanoid::LimbId::LIMB_HAND_LEFT));
    kinematics::set_max_values_hands_joints(humanoid::get_max_angles_of_limb(humanoid::LimbId::LIMB_HAND_LEFT));
    *(trajectory::current_l_l().pos) = _default_l_l_pos;
    *(trajectory::current_l_r().pos) = _default_l_r_pos;
    *(trajectory::current_h_l().pos) = _default_h_l_pos;
    *(trajectory::current_h_r().pos) = _default_h_r_pos;

    if (dynamixel::init(dynamixel::Dynamixel_config_t()) != 0)
        return InitResult::INIT_DYNAMIXEL_DRIVER_ERROR;

    const humanoid::LimbId limbs[humanoid::LIMB_COUNT]{humanoid::LIMB_LEG_LEFT, humanoid::LIMB_LEG_RIGHT, humanoid::LIMB_HAND_LEFT, humanoid::LIMB_HAND_RIGHT, humanoid::LIMB_HEAD};

    uint8_t drivers_count{0};
    std::vector<uint8_t> id_buff;
    for (uint8_t i = 0; i < humanoid::LIMB_COUNT; ++i)
    {
        humanoid::get_ids_of_limb(limbs[i], id_buff);
        drivers_count = dynamixel::checkConnections(id_buff);
        if (drivers_count != id_buff.size())
        {
            return static_cast<InitResult>(i + 1);
        }
    }

    drivers_count = dynamixel::setupDrivers(std::vector<uint8_t>(humanoid::drivers_id_buffer().begin(), humanoid::drivers_id_buffer().end()), 50);
    if (drivers_count != SERVO_COUNT)
        return InitResult::INIT_SETUP_DRIVERS_ERROR;

    // auto r = dynamixel::checkCamera();
    // if (r)
    //     return InitResult::INIT_CAMERA_NOT_FOUND_ERROR;

    // r = orientation::init();
    // if (r)
    //     return InitResult::INIT_ORIENT_ERROR;

    return InitResult::INIT_SUCC;
}

void take_start_pos()
{

    dynamixel::syncReadPosition(humanoid::drivers_id_buffer().data(), (int32_t *)humanoid::present_pos_buffer().data(), SERVO_COUNT);

    // ----------------------LEG_LEFT------------------------
    auto l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_LEG_LEFT);
    dynamixel::value_to_rad_arr(humanoid::present_pos_buffer().data() + l_range.first,
                                (double *)(trajectory::get_last_rads_buff().data() + l_range.first),
                                l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_LEFT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_LEFT));

    auto ik_result = kinematics::legIK(_default_l_l_pos, (kinematics::leg_t *)(trajectory::get_ik_rads_buff().data() + l_range.first), (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS));

    dynamixel::values_backlash_compensate_rad((const double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              (double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              l_range.second,
                                              dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);

    kinematics::calc_joint_velocity_by_time_arr(trajectory::get_last_rads_buff().data() + l_range.first,
                                                trajectory::get_ik_rads_buff().data() + l_range.first,
                                                (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                l_range.second, 2.0);

    dynamixel::rad_to_value_arr(trajectory::get_ik_rads_buff().data() + l_range.first,
                                (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first), l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_LEFT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_LEFT));

    //---------------------------------LEG_RIGHT----------------------
    l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_LEG_RIGHT);
    dynamixel::value_to_rad_arr(humanoid::present_pos_buffer().data() + l_range.first,
                                (double *)(trajectory::get_last_rads_buff().data() + l_range.first),
                                l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_RIGHT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_RIGHT));
    ik_result = kinematics::legIK(_default_l_r_pos, (kinematics::leg_t *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                  (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS | kinematics::IKCONFIG_MIRROR_OUT));

    dynamixel::values_backlash_compensate_rad((const double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              (double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              l_range.second,
                                              dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);
    kinematics::calc_joint_velocity_by_time_arr(trajectory::get_last_rads_buff().data() + l_range.first,
                                                trajectory::get_ik_rads_buff().data() + l_range.first,
                                                (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                l_range.second, 2.0);
    dynamixel::rad_to_value_arr(trajectory::get_ik_rads_buff().data() + l_range.first,
                                (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first), l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_RIGHT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_RIGHT));
    //----------------------------------HAND_LEFT-------------------------
    l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_HAND_LEFT);
    dynamixel::value_to_rad_arr(humanoid::present_pos_buffer().data() + l_range.first,
                                (double *)(trajectory::get_last_rads_buff().data() + l_range.first),
                                l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_RIGHT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_RIGHT));
    ik_result = kinematics::handIK(_default_h_l_pos, (kinematics::hand_t *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                   (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS | kinematics::IKCONFIG_MIRROR_OUT));

    dynamixel::values_backlash_compensate_rad((const double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              (double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              l_range.second,
                                              dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);
    kinematics::calc_joint_velocity_by_time_arr(trajectory::get_last_rads_buff().data() + l_range.first,
                                                trajectory::get_ik_rads_buff().data() + l_range.first,
                                                (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                l_range.second, 2.0);
    dynamixel::rad_to_value_arr(trajectory::get_ik_rads_buff().data() + l_range.first,
                                (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first), l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_LEFT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_LEFT));

    //----------------------------------HAND_RIGHT-------------------------
    l_range = humanoid::get_limb_range_in_buffers(humanoid::LimbId::LIMB_HAND_RIGHT);
    dynamixel::value_to_rad_arr(humanoid::present_pos_buffer().data() + l_range.first,
                                (double *)(trajectory::get_last_rads_buff().data() + l_range.first),
                                l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_RIGHT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_RIGHT));
    ik_result = kinematics::handIK(_default_h_r_pos, (kinematics::hand_t *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                   (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS));

    dynamixel::values_backlash_compensate_rad((const double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              (double *)(trajectory::get_ik_rads_buff().data() + l_range.first),
                                              l_range.second,
                                              dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);
    kinematics::calc_joint_velocity_by_time_arr(trajectory::get_last_rads_buff().data() + l_range.first,
                                                trajectory::get_ik_rads_buff().data() + l_range.first,
                                                (int32_t *)(humanoid::velocity_buffer().data() + l_range.first),
                                                l_range.second, 2.0);
    dynamixel::rad_to_value_arr(trajectory::get_ik_rads_buff().data() + l_range.first,
                                (int32_t *)(humanoid::goal_pos_buffer().data() + l_range.first), l_range.second,
                                humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_HAND_RIGHT),
                                humanoid::get_limbs_factor(humanoid::LimbId::LIMB_HAND_RIGHT));

    humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_HEAD, {512, 512});

    //-------------------------------------SEND-------------------------------

    dynamixel::syncWrite((uint8_t *)humanoid::drivers_id_buffer().data(), (int32_t *)humanoid::velocity_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);
    dynamixel::syncWrite((uint8_t *)humanoid::drivers_id_buffer().data(), (int32_t *)humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);
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
    SERIAL_OUT_L_THRSAFE("[SYSTEM] Init succ!");

    take_start_pos();

    osDelay(2500);
    for (;;)
    {
        loop();
        SERIAL_BEGIN
        if (serialEventRun)
            serialEventRun();
        SERIAL_END
    }
}

#if 1

uint8_t _i{0};
double speed_exec = 60; // mm/sec

uint8_t temp = 0;

void loop()
{
    // bool avail{false};
    // SERIAL_AVAIABLE_THRSAFE(avail);
    // if (avail && !trajectory::executable_l_l())
    // {
    //     //=======Read serial=========
    //     String strbuf;
    //     SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
    //     _l_l_pointer->pos->x = _l_r_pointer->pos->x = strbuf.toFloat();
    //     SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
    //     _l_l_pointer->pos->y = _l_r_pointer->pos->y = strbuf.toFloat();
    //     SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
    //     _l_l_pointer->pos->z = _l_r_pointer->pos->z = strbuf.toFloat();
    //     SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
    //     _l_l_pointer->pos->a = _l_r_pointer->pos->a = strbuf.toFloat() * DEG_TO_RAD;
    //     SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
    //     _l_l_pointer->pos->b = _l_r_pointer->pos->b = strbuf.toFloat() * DEG_TO_RAD;
    //     SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
    //     _l_l_pointer->pos->g = _l_r_pointer->pos->g = strbuf.toFloat() * DEG_TO_RAD;
    //     SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');

    //     speed_exec = strbuf.toFloat();

    //     *(_l_l_pointer->exec_time) = kinematics::get_motion_time_by_speed(*(trajectory::current_l_l().pos), *(_l_l_pointer->pos), speed_exec) * 1000.0;
    //     //*(_l_r_pointer->exec_time) = kinematics::get_motion_time_by_speed(*(trajectory::current_l_r().pos), *(_l_r_pointer->pos), speed_exec) * 1000;

    //     SERIAL_OUT_L_THRSAFE(*(_l_l_pointer->exec_time));

    //     trajectory::add_goal_point_leg_left(*(_l_l_pointer));
    //     // trajectory::add_goal_point_leg_right(*(_l_r_pointer));
    // }
    // osDelay(5);
    //=======Read serial=========

    *(_l_l_pointer->pos) = _base_l_l_time_line_1[_i].first;
    *(_l_l_pointer->exec_time) = _base_l_l_time_line_1[_i].second;

    *(_l_r_pointer->pos) = _base_l_r_time_line_1[_i].first;
    *(_l_r_pointer->exec_time) = _base_l_r_time_line_1[_i].second;

    // *(_h_l_pointer->pos) = _base_h_l_time_line[_i].first;
    // *(_h_l_pointer->exec_time) = _base_h_l_time_line[_i].second;

    // *(_h_r_pointer->pos) = _base_h_r_time_line[_i].first;
    // *(_h_r_pointer->exec_time) = _base_h_r_time_line[_i].second;

    trajectory::add_goal_point_leg_left(*(_l_l_pointer));
    trajectory::add_goal_point_leg_right(*(_l_r_pointer));
    // trajectory::add_goal_point_hand_left(*(_h_l_pointer));
    // trajectory::add_goal_point_hand_right(*(_h_r_pointer));
    // SERIAL_BEGIN;

    // SERIAL_OUT(_i)
    // SERIAL_OUT_L(":\t")

    // SERIAL_OUT_L(_base_l_l_time_line[_i].second);
    // SERIAL_OUT_L(_base_l_l_time_line[_i].first.x);
    // SERIAL_OUT('\t');
    // SERIAL_OUT_L(_base_l_r_time_line[_i].first.x);

    // SERIAL_OUT_L(_base_l_l_time_line[_i].first.y);
    // SERIAL_OUT('\t');
    // SERIAL_OUT_L(_base_l_r_time_line[_i].first.y);

    // SERIAL_OUT_L(_base_l_l_time_line[_i].first.z);
    // SERIAL_OUT('\t');
    // SERIAL_OUT_L(_base_l_r_time_line[_i].first.z);

    // SERIAL_OUT_L(_base_l_l_time_line[_i].first.a);
    // SERIAL_OUT('\t');
    // SERIAL_OUT_L(_base_l_r_time_line[_i].first.a);

    // SERIAL_OUT_L(_base_l_l_time_line[_i].first.b);
    // SERIAL_OUT('\t');
    // SERIAL_OUT_L(_base_l_r_time_line[_i].first.b);

    // SERIAL_OUT_L(_base_l_l_time_line[_i].first.g);
    // SERIAL_OUT('\t');
    // SERIAL_OUT_L(_base_l_r_time_line[_i].first.g);

    // auto result = kinematics::legIK(_base_l_l_time_line[_i].first, nullptr, (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS));
    // SERIAL_OUT("Left IK result: ")
    // SERIAL_OUT_L(result);

    // result = kinematics::legIK(_base_l_r_time_line[_i].first, nullptr, (kinematics::IKCalcConfig)(kinematics::IKCONF_CHECK_ANGLE_RANGE_EXCEED | kinematics::IKCONF_CHECK_UNREACHABLE_COORDS | kinematics::IKCONFIG_MIRROR_OUT));
    // SERIAL_OUT("Right IK result: ")
    // SERIAL_OUT_L(result);

    // SERIAL_OUT_L("==================");
    // SERIAL_END;
    // SERIAL_OUT_L_THRSAFE("Sended");
    while (trajectory::executed_l_l() || trajectory::executed_l_r() || trajectory::executed_h_l() || trajectory::executed_h_r())
    {
        osDelay(5);
    }
    _i++;
    if (_i >= ACTIONS_COUNT)
    {
        _i = 0;
    }
    // SERIAL_OUT_L_THRSAFE("Is null");
    // SERIAL_OUT_L_THRSAFE("Add next");
}

#endif

#pragma region hands ik new
#if 0

double speed_exec = 100; // mm/sec

void loop()
{
    bool avail{false};
    SERIAL_AVAIABLE_THRSAFE(avail);
    if (avail && !trajectory::executed_h_l() && !trajectory::executed_h_r())
    {
        //=======Read serial=========
        String strbuf;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _h_l_pointer->pos->a = _h_r_pointer->pos->a = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _h_l_pointer->pos->r = _h_r_pointer->pos->r = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _h_l_pointer->pos->z = _h_r_pointer->pos->z = strbuf.toFloat();

        *(_h_l_pointer->exec_time) = kinematics::get_motion_time_by_speed(*(trajectory::current_h_l().pos), *(_h_l_pointer->pos), speed_exec) * 1000.0;
        *(_h_r_pointer->exec_time) = kinematics::get_motion_time_by_speed(*(trajectory::current_h_r().pos), *(_h_r_pointer->pos), speed_exec) * 1000.0;

        // SERIAL_OUT_L_THRSAFE(*(_h_l_pointer.exec_time));

        trajectory::add_goal_point_hand_left(*(_h_l_pointer));
        trajectory::add_goal_point_hand_right(*(_h_r_pointer));
        // SERIAL_OUT_L_THRSAFE("Added");
        osDelay(*(_h_l_pointer->exec_time));
    }
    osDelay(5);
}
#endif
#pragma endregion

#pragma region legs ik new
#if 0

double speed_exec = 50; // mm/sec

void loop()
{
    bool avail{false};
    SERIAL_AVAIABLE_THRSAFE(avail);
    if (avail && !trajectory::executable_l_l() && !trajectory::executable_l_r())
    {
        //=======Read serial=========
        String strbuf;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _l_l_pointer->pos->x = _l_r_pointer->pos->x = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _l_l_pointer->pos->y = _l_r_pointer->pos->y = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _l_l_pointer->pos->z = _l_r_pointer->pos->z = strbuf.toFloat();
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _l_l_pointer->pos->a = _l_r_pointer->pos->a = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _l_l_pointer->pos->b = _l_r_pointer->pos->b = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        _l_l_pointer->pos->g = _l_r_pointer->pos->g = strbuf.toFloat() * DEG_TO_RAD;
        SERIAL_READ_STRING_UNTIL_THREAD_SAVE(strbuf, '/');
        speed_exec = strbuf.toFloat();

        *(_l_l_pointer->exec_time) = kinematics::get_motion_time_by_speed(*(trajectory::current_l_l().pos), *(_l_l_pointer->pos), speed_exec) * 1000;
        *(_l_r_pointer->exec_time) = kinematics::get_motion_time_by_speed(*(trajectory::current_l_r().pos), *(_l_r_pointer->pos), speed_exec) * 1000;

        SERIAL_OUT_L_THRSAFE(*(_l_l_pointer->exec_time));

        trajectory::add_goal_point_leg_left(*(_l_l_pointer));
        trajectory::add_goal_point_leg_right(*(_l_r_pointer));
    }
    osDelay(5);
}
#endif

#pragma endregion

#pragma region Gyro
#if 0

orientation::Orient_vector_t gyro;
orientation::Orient_vector_t gyro_vel;

void loop()
{
    // orientation::update();
    orientation::get_rpy(gyro);
    orientation::get_rpy_velocity(gyro_vel);
    SERIAL_BEGIN;
    SERIAL_OUT(gyro.x);
    SERIAL_OUT(',')
    SERIAL_OUT_L(gyro_vel.x);
    // SERIAL_OUT(',')
    // SERIAL_OUT_L(gyro.z);
    SERIAL_END;

    osDelay(ORIENT_UPDATE_PERIOD);
}

#endif
#pragma endregion

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
        // dynamixel::values_backlash_compensate_rad(pres_pos.data(), pres_pos.data(), LEG_DRIVERS_COUNT,
        //                                           dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 0);

        std::vector<int32_t> new_values(LEG_DRIVERS_COUNT, 0);

        dynamixel::values_backlash_compensate_rad(_out_left.data(), _out_left.data(), LEG_DRIVERS_COUNT,
                                                  dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);

        kinematics::calc_joint_velocity_by_time_arr(pres_pos.data(), _out_left.data(), new_values.data(), LEG_DRIVERS_COUNT, time_exec);
        humanoid::set_velocity_to_limb(humanoid::LimbId::LIMB_LEG_LEFT, new_values);

        // set goal
        dynamixel::rad_to_value_arr(_out_left.data(), new_values.data(), new_values.size(),
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_LEFT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_LEFT));
        humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_LEFT, new_values);

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
        // dynamixel::values_backlash_compensate_rad(pres_pos.data(), pres_pos.data(), LEG_DRIVERS_COUNT,
        //                                           dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 0);

        dynamixel::values_backlash_compensate_rad(_out_right.data(), _out_right.data(), LEG_DRIVERS_COUNT,
                                                  dynamixel::servo_backlash_comp_buffer().data() + l_range.first, 1);
        kinematics::calc_joint_velocity_by_time_arr(pres_pos.data(), _out_right.data(), new_values.data(), LEG_DRIVERS_COUNT, time_exec);
        humanoid::set_velocity_to_limb(humanoid::LimbId::LIMB_LEG_RIGHT, new_values);

        // send velocity
        dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::velocity_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_VELOCITY);

        // set goal
        dynamixel::rad_to_value_arr(_out_right.data(), new_values.data(), new_values.size(),
                                    humanoid::get_offsets_of_limb(humanoid::LimbId::LIMB_LEG_RIGHT),
                                    humanoid::get_limbs_factor(humanoid::LimbId::LIMB_LEG_RIGHT));

        humanoid::set_goal_pos_to_limb(humanoid::LimbId::LIMB_LEG_RIGHT, new_values);

        // send values
        dynamixel::syncWrite(humanoid::drivers_id_buffer().data(), humanoid::goal_pos_buffer().data(), SERVO_COUNT, dynamixel::SyncWriteParamType::SYNCWRITE_POSITION);

        tposlast = tpos;
    }
    osDelay(5);
}
#endif
#pragma endregion