#pragma once

#include "humanoid.h"

#include <algorithm>

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
    };

    void humanoid_register_rtos()
    {
        osMutexDef(_mut_goal_pos_buf);
        osMutexCreate(osMutex(_mut_goal_pos_buf));

        osMutexDef(_mut_present_pos_buf);
        osMutexCreate(osMutex(_mut_present_pos_buf));

        osMutexDef(_mut_velocity_buf);
        osMutexCreate(osMutex(_mut_velocity_buf));
    }

    void lock_goal_pos_buffer_mutex()
    {
        osMutexWait(_mut_goal_pos_buf, portMAX_DELAY);
    }

    void unlock_goal_pos_buffer_mutex()
    {
        osMutexRelease(_mut_goal_pos_buf);
    }

    const std::array<int32_t, SERVO_COUNT> &goal_pos_buffer()
    {
        return drivers_goal_pos_buffer;
    }

    void lock_present_pos_buffer_mutex()
    {
        osMutexWait(_mut_present_pos_buf, portMAX_DELAY);
    }

    void unlock_present_pos_buffer_mutex()
    {
        osMutexRelease(_mut_present_pos_buf);
    }

    const std::array<int32_t, SERVO_COUNT> &present_pos_buffer()
    {
        return drivers_present_pos_buffer;
    }

    void lock_velocity_buffer_mutex()
    {
        osMutexWait(_mut_velocity_buf, portMAX_DELAY);
    }
    void unlock_velocity_buffer_mutex()
    {
        osMutexRelease(_mut_velocity_buf);
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
        return {(uint8_t)limb * LEG_DRIVERS_COUNT, LEG_DRIVERS_COUNT};
#else
        if (limb < LimbId::LIMB_LEG_RIGHT)
        {
            return {(uint8_t)limb * LEG_DRIVERS_COUNT, LEG_DRIVERS_COUNT};
        }
        else
        {
            return {2 * LEG_DRIVERS_COUNT + ((uint8_t)limb - 2) * HAND_DRIVERS_COUNT, HAND_DRIVERS_COUNT};
        }
#endif
    }

    uint8_t get_buf_id_by_path(const struct DriverPath_t &dpath)
    {
        uint8_t did{0};
#if LEG_DRIVERS_COUNT == HAND_DRIVERS_COUNT
        did = dpath.limb_id * LEG_DRIVERS_COUNT + dpath.joint_id;
#else
        if (dpath.limb_id < 2)
            did = dpath.limb_id * LEG_DRIVERS_COUNT + dpath.joint_id;
        else if (dpath.limb_id < 4)
            did = 2 * LEG_DRIVERS_COUNT + (dpath.limb_id - 2) * HAND_DRIVERS_COUNT + dpath.joint_id;
#endif
        return did;
    }

    uint8_t get_driver_id_by_path(const struct DriverPath_t &dpath)
    {
        return drivers_ids[get_buf_id_by_path(dpath)];
    }

    uint8_t get_ids_of_limb(LimbId limb, std::vector<uint8_t> &out)
    {
        if (limb == LimbId::LIMB_LEG_LEFT)
        {
            out.assign(drivers_ids.begin(), drivers_ids.begin() + LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
        else if (limb == LimbId::LIMB_LEG_RIGHT)
        {
            out.assign(drivers_ids.begin() + LEG_DRIVERS_COUNT, drivers_ids.begin() + 2 * LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
    }

    uint8_t get_goal_pos_of_limb(LimbId limb, std::vector<int32_t> &out)
    {
        if (limb == LimbId::LIMB_LEG_LEFT)
        {
            out.assign(drivers_goal_pos_buffer.begin(), drivers_goal_pos_buffer.begin() + LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
        else if (limb == LimbId::LIMB_LEG_RIGHT)
        {
            out.assign(drivers_goal_pos_buffer.begin() + LEG_DRIVERS_COUNT, drivers_goal_pos_buffer.begin() + 2 * LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
    }

    uint8_t get_present_pos_of_limb(LimbId limb, std::vector<int32_t> &out)
    {
        if (limb == LimbId::LIMB_LEG_LEFT)
        {
            out.assign(drivers_present_pos_buffer.begin(), drivers_present_pos_buffer.begin() + LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
        else if (limb == LimbId::LIMB_LEG_RIGHT)
        {
            out.assign(drivers_present_pos_buffer.begin() + LEG_DRIVERS_COUNT, drivers_present_pos_buffer.begin() + 2 * LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
    }

    uint8_t get_velocity_of_limb(LimbId limb, std::vector<int32_t> &out)
    {
        if (limb == LimbId::LIMB_LEG_LEFT)
        {
            out.assign(drivers_velocity_buffer.begin(), drivers_velocity_buffer.begin() + LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
        else if (limb == LimbId::LIMB_LEG_RIGHT)
        {
            out.assign(drivers_velocity_buffer.begin() + LEG_DRIVERS_COUNT, drivers_velocity_buffer.begin() + 2 * LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
    }

    uint8_t set_goal_pos_to_limb(LimbId limb, const std::vector<int32_t> &source)
    {
        if (limb == LimbId::LIMB_LEG_LEFT)
        {
            std::copy(source.begin(), source.begin() + LEG_DRIVERS_COUNT, drivers_goal_pos_buffer.begin());
            return LEG_DRIVERS_COUNT;
        }
        else if (limb == LimbId::LIMB_LEG_RIGHT)
        {
            std::copy(source.begin(), source.begin() + LEG_DRIVERS_COUNT, drivers_goal_pos_buffer.begin() + LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
    }

    uint8_t set_velocity_to_limb(LimbId limb, const std::vector<int32_t> &source)
    {
        if (limb == LimbId::LIMB_LEG_LEFT)
        {
            std::copy(source.begin(), source.begin() + LEG_DRIVERS_COUNT, drivers_velocity_buffer.begin());
            return LEG_DRIVERS_COUNT;
        }
        else if (limb == LimbId::LIMB_LEG_RIGHT)
        {
            std::copy(source.begin(), source.begin() + LEG_DRIVERS_COUNT, drivers_velocity_buffer.begin() + LEG_DRIVERS_COUNT);
            return LEG_DRIVERS_COUNT;
        }
    }

}
