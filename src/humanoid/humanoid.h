#pragma once

#include <Arduino.h>
#include <RTOS.h>

#include "../../construction.h"
#include "../../config.h"

#include <inttypes.h>
#include <vector>
#include <array>

namespace humanoid
{

    typedef enum LimbId : uint8_t
    {
        LIMB_LEG_LEFT = 0,
        LIMB_LEG_RIGHT = 1,
        LIMB_UNDEFINED = 255,
    };

    struct DriverPath_t
    {
        LimbId limb_id;
        uint8_t joint_id;
        DriverPath_t(LimbId _limb_id = LimbId::LIMB_UNDEFINED, uint8_t _joint_id = 255) : limb_id(_limb_id), joint_id(_joint_id) {}
    };

    void humanoid_register_rtos();

    // =========Buffers==========

    osStatus lock_goal_pos_buffer_mutex(uint32_t);
    osStatus unlock_goal_pos_buffer_mutex();
    const std::array<int32_t, SERVO_COUNT> &goal_pos_buffer();

    osStatus lock_present_pos_buffer_mutex(uint32_t);
    osStatus unlock_present_pos_buffer_mutex();
    const std::array<int32_t, SERVO_COUNT> &present_pos_buffer();

    osStatus lock_velocity_buffer_mutex(uint32_t);
    osStatus unlock_velocity_buffer_mutex();
    const std::array<int32_t, SERVO_COUNT> &velocity_buffer();

    const std::array<uint8_t, SERVO_COUNT> &drivers_id_buffer();

    std::pair<uint8_t, uint8_t> get_limb_range_in_buffers(LimbId limb);

    const double *get_min_angles_of_limb(LimbId limb);
    const double *get_max_angles_of_limb(LimbId limb);
    const double *get_offsets_of_limb(LimbId limb);

    int8_t get_limbs_offset_factor(LimbId limb);

    //========Buffers operations======

    uint8_t get_buf_id_by_path(const struct DriverPath_t &dpath);

    uint8_t get_driver_id_by_path(const struct DriverPath_t &dpath);

    uint8_t get_ids_of_limb(LimbId limb, std::vector<uint8_t> &out);

    uint8_t get_goal_pos_of_limb(LimbId limb, int32_t *out);

    uint8_t get_present_pos_of_limb(LimbId limb, int32_t *out);

    uint8_t get_velocity_of_limb(LimbId limb, int32_t *out);

    uint8_t set_goal_pos_to_limb(LimbId limb, const std::vector<int32_t> &source);

    uint8_t set_present_pos_to_limb(LimbId limb, const std::vector<int32_t> &source);

    uint8_t set_velocity_to_limb(LimbId limb, const std::vector<int32_t> &source);
}
