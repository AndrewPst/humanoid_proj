#pragma once

#include "humanoid.h"

#include <algorithm>
#include <utility>

namespace humanoid
{
    osMutexId _mut_goal_pos_buf{nullptr};
    osMutexId _mut_present_pos_buf{nullptr};
    osMutexId _mut_velocity_buf{nullptr};

    std::array<int32_t, SERVO_COUNT> drivers_goal_pos_buffer;
    std::array<int32_t, SERVO_COUNT> drivers_present_pos_buffer;
    std::array<int32_t, SERVO_COUNT> drivers_velocity_buffer;

    const std::array<uint8_t, SERVO_COUNT> drivers_ids{
        LEG_LEFT_0_DRIVER_ID,
        LEG_LEFT_1_DRIVER_ID,
        LEG_LEFT_2_DRIVER_ID,
        LEG_LEFT_3_DRIVER_ID,
        LEG_LEFT_4_DRIVER_ID,
        LEG_LEFT_5_DRIVER_ID,
        LEG_RIGHT_0_DRIVER_ID,
        LEG_RIGHT_1_DRIVER_ID,
        LEG_RIGHT_2_DRIVER_ID,
        LEG_RIGHT_3_DRIVER_ID,
        LEG_RIGHT_4_DRIVER_ID,
        LEG_RIGHT_5_DRIVER_ID,
        HAND_LEFT_0_DRIVER_ID,
        HAND_LEFT_1_DRIVER_ID,
        HAND_LEFT_2_DRIVER_ID,
        HAND_RIGHT_0_DRIVER_ID,
        HAND_RIGHT_1_DRIVER_ID,
        HAND_RIGHT_2_DRIVER_ID,
        HEAD_0_DRIVER_ID,
        HEAD_1_DRIVER_ID,
    };

    const std::array<double, LEG_DRIVERS_COUNT + HAND_DRIVERS_COUNT + HEAD_DRIVERS_COUNT> joints_offsets{
        LEG_OFFSET_ANGLE_0_SERVO,
        LEG_OFFSET_ANGLE_1_SERVO,
        LEG_OFFSET_ANGLE_2_SERVO,
        LEG_OFFSET_ANGLE_3_SERVO,
        LEG_OFFSET_ANGLE_4_SERVO,
        LEG_OFFSET_ANGLE_5_SERVO,
        HAND_OFFSET_ANGLE_0_SERVO,
        HAND_OFFSET_ANGLE_1_SERVO,
        HAND_OFFSET_ANGLE_2_SERVO,
        HEAD_OFFSET_ANGLE_0_SERVO,
        HEAD_OFFSET_ANGLE_1_SERVO,
    };

    const std::array<double, LEG_DRIVERS_COUNT + HAND_DRIVERS_COUNT + HEAD_DRIVERS_COUNT> joints_min_values{
        LEG_MIN_ANGLE_0_SERVO,
        LEG_MIN_ANGLE_1_SERVO,
        LEG_MIN_ANGLE_2_SERVO,
        LEG_MIN_ANGLE_3_SERVO,
        LEG_MIN_ANGLE_4_SERVO,
        LEG_MIN_ANGLE_5_SERVO,
        HAND_MIN_ANGLE_0_SERVO,
        HAND_MIN_ANGLE_1_SERVO,
        HAND_MIN_ANGLE_2_SERVO,
        HEAD_MIN_ANGLE_0_SERVO,
        HEAD_MIN_ANGLE_1_SERVO,
    };

    const std::array<double, LEG_DRIVERS_COUNT + HAND_DRIVERS_COUNT + HEAD_DRIVERS_COUNT> joints_max_values{
        LEG_MAX_ANGLE_0_SERVO,
        LEG_MAX_ANGLE_1_SERVO,
        LEG_MAX_ANGLE_2_SERVO,
        LEG_MAX_ANGLE_3_SERVO,
        LEG_MAX_ANGLE_4_SERVO,
        LEG_MAX_ANGLE_5_SERVO,
        HAND_MAX_ANGLE_0_SERVO,
        HAND_MAX_ANGLE_1_SERVO,
        HAND_MAX_ANGLE_2_SERVO,
        HEAD_MAX_ANGLE_0_SERVO,
        HEAD_MAX_ANGLE_1_SERVO,
    };

    const double *get_min_angles_of_limb(LimbId limb)
    {
        if (limb <= LimbId::LIMB_LEG_RIGHT)
            return joints_min_values.data();
        if (limb <= LimbId::LIMB_HAND_RIGHT)
            return joints_min_values.data() + LEG_DRIVERS_COUNT;
        if (limb == LimbId::LIMB_HEAD)
            return joints_min_values.data() + LEG_DRIVERS_COUNT + HAND_DRIVERS_COUNT;
    }

    const double *get_max_angles_of_limb(LimbId limb)
    {
        if (limb <= LimbId::LIMB_LEG_RIGHT)
            return joints_max_values.data();
        if (limb <= LimbId::LIMB_HAND_RIGHT)
            return joints_max_values.data() + LEG_DRIVERS_COUNT;
        if (limb == LimbId::LIMB_HEAD)
            return joints_min_values.data() + LEG_DRIVERS_COUNT + HAND_DRIVERS_COUNT;
    }

    const double *get_offsets_of_limb(LimbId limb)
    {
        if (limb <= LimbId::LIMB_LEG_RIGHT)
            return joints_offsets.data();
        if (limb <= LimbId::LIMB_HAND_RIGHT)
            return joints_offsets.data() + LEG_DRIVERS_COUNT;
        if (limb == LimbId::LIMB_HEAD)
            return joints_min_values.data() + LEG_DRIVERS_COUNT + HAND_DRIVERS_COUNT;
    }

    void humanoid_register_rtos()
    {
        osMutexDef(_mut_goal_pos_buf);
        _mut_goal_pos_buf = osMutexCreate(osMutex(_mut_goal_pos_buf));

        osMutexDef(_mut_present_pos_buf);
        _mut_present_pos_buf = osMutexCreate(osMutex(_mut_present_pos_buf));

        osMutexDef(_mut_velocity_buf);
        _mut_velocity_buf = osMutexCreate(osMutex(_mut_velocity_buf));
    }

    osStatus lock_goal_pos_buffer_mutex(uint32_t lock_time = portMAX_DELAY)
    {
        return osMutexWait(_mut_goal_pos_buf, lock_time);
    }

    osStatus unlock_goal_pos_buffer_mutex()
    {
        return osMutexRelease(_mut_goal_pos_buf);
    }

    const std::array<int32_t, SERVO_COUNT> &goal_pos_buffer()
    {
        return drivers_goal_pos_buffer;
    }

    osStatus lock_present_pos_buffer_mutex(uint32_t lock_time = portMAX_DELAY)
    {
        return osMutexWait(_mut_present_pos_buf, lock_time);
    }

    osStatus unlock_present_pos_buffer_mutex()
    {
        return osMutexRelease(_mut_present_pos_buf);
    }

    const std::array<int32_t, SERVO_COUNT> &present_pos_buffer()
    {
        return drivers_present_pos_buffer;
    }

    osStatus lock_velocity_buffer_mutex(uint32_t lock_time = portMAX_DELAY)
    {
        return osMutexWait(_mut_velocity_buf, lock_time);
    }

    osStatus unlock_velocity_buffer_mutex()
    {
        return osMutexRelease(_mut_velocity_buf);
    }

    const std::array<int32_t, SERVO_COUNT> &velocity_buffer()
    {
        return drivers_velocity_buffer;
    }

    const std::array<uint8_t, SERVO_COUNT> &drivers_id_buffer()
    {
        return drivers_ids;
    }

    std::pair<uint8_t, uint8_t> get_limb_range_in_buffers(LimbId limb)
    {
#if LEG_DRIVERS_COUNT == HAND_DRIVERS_COUNT
        return {(uint8_t)limb * LEG_DRIVERS_COUNT, limb == LimbId::LIMB_HEAD ? HEAD_DRIVERS_COUNT : LEG_DRIVERS_COUNT};
#else
        if (limb <= LimbId::LIMB_LEG_RIGHT)
        {
            return {(uint8_t)limb * LEG_DRIVERS_COUNT, LEG_DRIVERS_COUNT};
        }
        else if (limb <= LimbId::LIMB_HAND_RIGHT)
        {
            return {2 * LEG_DRIVERS_COUNT + ((uint8_t)limb - 2) * HAND_DRIVERS_COUNT, HAND_DRIVERS_COUNT};
        }
        else if (limb == LimbId::LIMB_HEAD)
        {
            return {2 * LEG_DRIVERS_COUNT + 2 * HAND_DRIVERS_COUNT, HEAD_DRIVERS_COUNT};
        }
        return {0, 0};
#endif
    }

    uint8_t get_buf_id_by_path(const struct DriverPath_t &dpath)
    {
        uint8_t did{0};
#if LEG_DRIVERS_COUNT == HAND_DRIVERS_COUNT
        did = dpath.limb_id * LEG_DRIVERS_COUNT + dpath.joint_id;
#else
        if (dpath.limb_id <= LimbId::LIMB_LEG_RIGHT)
        {
            did = dpath.limb_id * LEG_DRIVERS_COUNT + dpath.joint_id;
        }
        else if (dpath.limb_id <= LimbId::LIMB_HAND_RIGHT)
        {
            did = 2 * LEG_DRIVERS_COUNT + (dpath.limb_id - 2) * HAND_DRIVERS_COUNT + dpath.joint_id;
        }
        if (dpath.limb_id == LimbId::LIMB_HEAD)
        {
            did = 2 * LEG_DRIVERS_COUNT + 2 * HAND_DRIVERS_COUNT + dpath.joint_id;
        }
#endif
        return did;
    }

    int8_t get_limbs_factor(LimbId limb)
    {
        if (limb == LimbId::LIMB_LEG_LEFT || limb == LimbId::LIMB_HAND_LEFT || limb == LimbId::LIMB_HEAD)
            return 1;
        else
            return -1;
    }

    uint8_t get_driver_id_by_path(const struct DriverPath_t &dpath)
    {
        return drivers_ids[get_buf_id_by_path(dpath)];
    }

    uint8_t get_ids_of_limb(LimbId limb, std::vector<uint8_t> &out)
    {
        auto l_range = get_limb_range_in_buffers(limb);
        if (l_range.second == 0)
            return 0;
        out.assign(drivers_ids.begin() + l_range.first, drivers_ids.begin() + l_range.first + l_range.second);
        return l_range.second;
    }

    uint8_t get_goal_pos_of_limb(LimbId limb, int32_t *out)
    {
        auto l_range = get_limb_range_in_buffers(limb);
        if (l_range.second == 0)
            return 0;
        lock_goal_pos_buffer_mutex();
        std::copy(drivers_goal_pos_buffer.begin() + l_range.first, drivers_goal_pos_buffer.begin() + l_range.first + l_range.second, out);
        unlock_goal_pos_buffer_mutex();
        return l_range.second;
    }

    uint8_t get_present_pos_of_limb(LimbId limb, int32_t *out)
    {
        auto l_range = get_limb_range_in_buffers(limb);
        if (l_range.second == 0)
            return 0;
        lock_present_pos_buffer_mutex();
        std::copy(drivers_present_pos_buffer.begin(), drivers_present_pos_buffer.begin() + LEG_DRIVERS_COUNT, out);
        unlock_present_pos_buffer_mutex();
        return l_range.second;
    }

    uint8_t get_velocity_of_limb(LimbId limb, int32_t *out)
    {
        auto l_range = get_limb_range_in_buffers(limb);
        if (l_range.second == 0)
            return 0;
        lock_velocity_buffer_mutex();
        std::copy(drivers_velocity_buffer.begin(), drivers_velocity_buffer.begin() + LEG_DRIVERS_COUNT, out);
        unlock_velocity_buffer_mutex();
        return l_range.second;
    }

    uint8_t set_goal_pos_to_limb(LimbId limb, const std::vector<int32_t> &source)
    {
        auto l_range = get_limb_range_in_buffers(limb);
        if (l_range.second == 0)
            return 0;
        lock_goal_pos_buffer_mutex();
        std::copy(source.begin(), source.begin() + l_range.second, drivers_goal_pos_buffer.begin() + l_range.first);
        unlock_goal_pos_buffer_mutex();
        return l_range.second;
    }

    uint8_t set_present_pos_to_limb(LimbId limb, const std::vector<int32_t> &source)
    {
        auto l_range = get_limb_range_in_buffers(limb);
        if (l_range.second == 0)
            return 0;
        lock_present_pos_buffer_mutex();
        std::copy(source.begin(), source.begin() + l_range.second, drivers_present_pos_buffer.begin() + l_range.first);
        unlock_present_pos_buffer_mutex();
        return l_range.second;
    }

    uint8_t set_velocity_to_limb(LimbId limb, const std::vector<int32_t> &source)
    {
        auto l_range = get_limb_range_in_buffers(limb);
        if (l_range.second == 0)
            return 0;
        lock_velocity_buffer_mutex();
        std::copy(source.begin(), source.begin() + l_range.second, drivers_velocity_buffer.begin() + l_range.first);
        unlock_velocity_buffer_mutex();
        return l_range.second;
    }

}
