#pragma once

#include <Arduino.h>
#include <inttypes.h>
#include "DynamixelWorkbench.h"
#include "../../config.h"
#include <vector>
#include <array>

#if SERVO_TYPE == AX_12A
#define RAD2VALUE(a) (((a + 5.0 * M_PI / 6.0) * 1024.0) / (10.0 * M_PI / 6.0))
#define VALUE2RAD(v) ((v * (10.0 * M_PI / 6.0)) / 1024 - 5.0 * M_PI / 6.0)
#endif

namespace dynamixel
{

    namespace
    {
        DynamixelWorkbench dxl;

        std::array<double, SERVO_COUNT> newPosBuf;
    }

    enum SyncWriteParamType : uint8_t
    {
        SYNCWRITE_POSITION = 0,
        SYNCWRITE_VELOCITY = 1,
        SYNCWRITE_ACCELERATION = 2
    };

    struct Dynamixel_config_t
    {
        uint32_t boudrate{1000000};
        const char *device_name{"3"};
    };

    void thread_dynamixel(void const *);

    void dynamixel_register_rtos();

    uint8_t init(const struct Dynamixel_config_t &);

    /// @brief check the connected servos and count him
    /// @param out array of id connected servos
    /// @param max_servo_count set the limit of connected servos
    /// @return count of connected servos
    uint8_t checkConnections(std::vector<uint8_t> *out = nullptr, uint8_t max_servo_count = 255);

    uint8_t setupDrivers(const std::vector<uint8_t> &, int32_t, int32_t);

    uint8_t syncReadPosition(const std::vector<uint8_t> &ids, std::vector<int32_t> &out); // Very slow function with use the Dynamixex protocol 1.0

    uint8_t syncWrite(const std::vector<uint8_t> &ids, const std::vector<int32_t> &data, SyncWriteParamType paramType);

    // uint8_t syncWritePosition(const std::vector<uint8_t> &ids, const std::vector<int32_t> &data);

    // uint8_t syncWriteVelocity(const std::vector<uint8_t> &ids, const std::vector<int32_t> &data);
};