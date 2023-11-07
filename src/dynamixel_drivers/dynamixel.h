#pragma once

#include <Arduino.h>
#include <inttypes.h>
#include "DynamixelWorkbench.h"
#include "../../config.h"
#include "../../construction.h"
#include <vector>
#include <array>

#if SERVO_TYPE == AX_12A
#define RAD2VALUE(a) (((a + 5.0 * M_PI / 6.0) * 1024.0) / (10.0 * M_PI / 6.0))
#define VALUE2RAD(v) ((v * (10.0 * M_PI / 6.0)) / 1024 - 5.0 * M_PI / 6.0)
#else
#error Servo type not supported
#endif

/// @brief functuons in this namespace exercise control over the connected servos
namespace dynamixel
{

    enum SyncWriteParamType : uint8_t
    {
        SYNCWRITE_POSITION = 0,
        SYNCWRITE_VELOCITY = 1,
        SYNCWRITE_ACCELERATION = 2
    };

    struct Dynamixel_config_t
    {
        uint32_t boudrate{1000000};
        const char *device_name{"3"}; // for openCR - "3", for openCM - " "
    };

    /// @brief register mutexes
    void dynamixel_register_rtos();

    /// @brief initializate dynamixel driver
    /// @param config Initialization parameters
    /// @return 0 - successfull, 1 - init error
    uint8_t init(const struct Dynamixel_config_t &config);

    /// @brief check the connected devices and count him
    /// @param out array of id connected devices
    /// @param max_servo_count set the limit of connected devices
    /// @return count of connected devices
    uint8_t findConnections(std::vector<uint8_t> *out = nullptr, uint8_t max_servo_count = 255);

    /// @brief check for connected devices by the specified IDs
    /// @param ids array of IDs
    /// @return count of connected devices
    uint8_t checkConnections(const std::vector<uint8_t> &ids);

    /// @brief Check the camera connection with CAM_ID
    /// @return 0 - succ, other - not found
    uint8_t checkCamera();

    /// @brief Connect to servos and setup handlers. Call this function once
    /// @param ids array of connected servos
    /// @param default_velocity default velocity of servos
    /// @return count of successfully configurated servos
    uint8_t setupDrivers(const std::vector<uint8_t> &ids, int32_t default_velocity);

    /// @brief sequential reading of servos present positions
    /// @param ids IDs of servos buffer
    /// @param out result buffer (in dynamixel units)
    /// @param count IDs count
    /// @return 0 - successfull, another - error
    uint8_t syncReadPosition(const uint8_t *ids, int32_t *out, uint8_t count);

    /// @brief sequential reading of servos present positions
    /// @param ids vector of ids
    /// @param out vector of results
    /// @return 0 - successfull, another - error
    uint8_t syncReadPosition(const std::vector<uint8_t> &ids, std::vector<int32_t> &out); // Very slow function with use the Dynamixex protocol 1.0

    /// @brief sync write data to servos
    /// @param ids IDs of servos
    /// @param data data to send
    /// @param paramType type of data being send
    /// @return 0 - successful, another - error
    uint8_t syncWrite(const std::vector<uint8_t> &ids, const std::vector<int32_t> &data, SyncWriteParamType paramType);

    /// @brief sync write data to servos
    /// @param ids IDs of servos
    /// @param data data to send
    /// @param count IDs count
    /// @param paramType type of data being send
    /// @return 0 - successful, another - error
    uint8_t syncWrite(const uint8_t *ids, const int32_t *data, uint8_t count, SyncWriteParamType paramType);

    /// @brief convert everyone position in rad of servo to position in dynamixel unit
    /// @param in angles in rad
    /// @param out angless in dynamixel units
    /// @param count count of data
    /// @param offsets offsets buffer for everyone servo (in rad)
    /// @param offset_factor offset direction (+ or -)
    /// @return count of converted data
    uint8_t rad_to_value_arr(const double *in, int32_t *out, uint8_t count,
                             const double *offsets = nullptr, int8_t offset_factor = 1);

    /// @brief read registers of camera
    /// @param address address of object (16 + <address> * 16)
    /// @param out 16 byte array for camera answer
    /// @return 0 - succ, other - err
    uint8_t readCumRegister(uint16_t address, uint8_t *out);

    /// @brief convert everyone position in dynamixel unit of servo to position in rad
    /// @param in angles in dynamixel units
    /// @param out angles in rad
    /// @param count count of data
    /// @param offsets offsets buffer for everyone servo
    /// @param offset_factor offset direction (+ || -)
    /// @return count of converted data
    uint8_t value_to_rad_arr(const int32_t *in, double *out, uint8_t count,
                             const double *offsets = nullptr, int8_t offset_factor = 1);

    uint8_t values_backlash_compensate_rad(const double *in, double *out, uint8_t size, const double *backlash_com_arr, uint8_t mode);

    const std::array<double, SERVO_COUNT> &servo_backlash_comp_buffer();

};