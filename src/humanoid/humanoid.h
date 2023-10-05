#pragma once

#include <Arduino.h>
#include <RTOS.h>

#include "../../construction.h"
#include "../../config.h"

#include <inttypes.h>
#include <vector>
#include <array>

/// @brief this namespace is responsible for storing the attributes of joints in the form of a specific humanoid robot structure
namespace humanoid
{

    /// @brief id of humanoid limbs
    typedef enum LimbId : uint8_t
    {
        LIMB_LEG_LEFT = 0,
        LIMB_LEG_RIGHT = 1,
        LIMB_UNDEFINED = 255,
    };

    /// @brief Path of joint in humanoid structure
    struct DriverPath_t
    {
        LimbId limb_id;
        uint8_t joint_id;
        DriverPath_t(LimbId _limb_id = LimbId::LIMB_UNDEFINED, uint8_t _joint_id = 255) : limb_id(_limb_id), joint_id(_joint_id) {}
    };

    /// @brief register mutexes for buffers
    void humanoid_register_rtos();

    // =========Buffers==========

    osStatus lock_goal_pos_buffer_mutex(uint32_t);
    osStatus unlock_goal_pos_buffer_mutex();

    /// @brief Get array of goal positions (in dynamixel units) of all joints. Use (un)lock_goal_pos_buffer_mutex before read/write to buffer
    /// @return array with structure: [LEG_LEFT[0, 5], LEG_RIGHT[0, 5], HAND_LEFT[0, 2], HAND_RIGHT[0, 2], HEAD[0, 1]]
    const std::array<int32_t, SERVO_COUNT> &goal_pos_buffer();

    osStatus lock_present_pos_buffer_mutex(uint32_t);
    osStatus unlock_present_pos_buffer_mutex();

    /// @brief Get array of present positions (in dynamixel units) of all joints. Use (un)lock_present_pos_buffer_mutex before read/write to buffer
    /// @return array with structure: [LEG_LEFT[0, 5], LEG_RIGHT[0, 5], HAND_LEFT[0, 2], HAND_RIGHT[0, 2], HEAD[0, 1]]
    const std::array<int32_t, SERVO_COUNT> &present_pos_buffer();

    osStatus lock_velocity_buffer_mutex(uint32_t);
    osStatus unlock_velocity_buffer_mutex();

    /// @brief Get array of velocity (in dynamixel units) of all joints. Use (un)lock_velocity_buffer_mutex before read/write to buffer
    /// @return array with structure: [LEG_LEFT[0, 5], LEG_RIGHT[0, 5], HAND_LEFT[0, 2], HAND_RIGHT[0, 2], HEAD[0, 1]]
    const std::array<int32_t, SERVO_COUNT> &velocity_buffer();

    /// @brief Get array of ids servos
    /// @return array with structure: [LEG_LEFT[0, 5], LEG_RIGHT[0, 5], HAND_LEFT[0, 2], HAND_RIGHT[0, 2], HEAD[0, 1]]
    const std::array<uint8_t, SERVO_COUNT> &drivers_id_buffer();

    /// @brief get the address of limb in buffers
    /// @param limb limb id
    /// @return .first - offset from beginning buffer, .second - count of elements
    std::pair<uint8_t, uint8_t> get_limb_range_in_buffers(LimbId limb);

    /// @brief get array of min angles (in radians) of limb
    /// @param limb limb id
    /// @return address of first element of buffers
    const double *get_min_angles_of_limb(LimbId limb);

    /// @brief get array of max angles (in radians) of limb
    /// @param limb limb id
    /// @return address of first element of buffers
    const double *get_max_angles_of_limb(LimbId limb);

    /// @brief get array of offset angles (in radians) of limb
    /// @param limb limb id
    /// @return address of first element of buffers
    const double *get_offsets_of_limb(LimbId limb);

    /// @brief get limbs factor
    /// @param limb limb id
    /// @return +1 if limb left, -1 if limb is right
    int8_t get_limbs_factor(LimbId limb);

    //========Buffers operations======

    /// @brief get joint id in buffers by joint path
    /// @param dpath joint path
    /// @return joint id in buffer
    uint8_t get_buf_id_by_path(const struct DriverPath_t &dpath);

    /// @brief return joint id by path
    /// @param dpath joint path
    /// @return joint id
    uint8_t get_driver_id_by_path(const struct DriverPath_t &dpath);

    /// @brief get joints ids in limb
    /// @param limb limb id
    /// @param out array of joints ids
    /// @return count of ids
    uint8_t get_ids_of_limb(LimbId limb, std::vector<uint8_t> &out);

    /// @brief get goal positions of joints in limb
    /// @param limb limb id
    /// @param out buffer of goal positions
    /// @return count of elements in out buffer
    uint8_t get_goal_pos_of_limb(LimbId limb, int32_t *out);

    /// @brief get present positions of joints in limb
    /// @param limb limb id
    /// @param out buffer of present positions
    /// @return count of elements in out buffer
    uint8_t get_present_pos_of_limb(LimbId limb, int32_t *out);

    /// @brief get velocity of joints in limb
    /// @param limb limb id
    /// @param out buffer of velocity
    /// @return count of elements in out buffer
    uint8_t get_velocity_of_limb(LimbId limb, int32_t *out);

    /// @brief set goal position to limbs joints
    /// @param limb limb id
    /// @param soure goal positions (int dynamixel units)
    /// @return count of elements in out buffer
    uint8_t set_goal_pos_to_limb(LimbId limb, const std::vector<int32_t> &source);

    /// @brief set present position to limbs joints
    /// @param limb limb id
    /// @param soure present positions (int dynamixel units)
    /// @return count of elements in out buffer
    uint8_t set_present_pos_to_limb(LimbId limb, const std::vector<int32_t> &source);

    /// @brief set velocity to limbs joints
    /// @param limb limb id
    /// @param soure velocity (int dynamixel units)
    /// @return count of elements in out buffer
    uint8_t set_velocity_to_limb(LimbId limb, const std::vector<int32_t> &source);
}
