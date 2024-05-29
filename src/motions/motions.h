#pragma once

#include <Arduino.h>
#include <inttypes.h>
#include <array>
#include <memory>

#include "../dynamics/trajectory.h"

namespace motions
{
    enum MotionKey : uint8_t
    {
        MOTION_FORWARD,
        MOTION_FALLING_BACK,
        MOTION_FALLING_FRONT,
        MOTIONS_COUNT,
        MOTION_UNKNOWN,
    };

    enum MotionStatus : uint8_t
    {
        M_STATUS_ENDED = 0,
        M_STATUS_IN_EXECUTION,
        M_STATUS_EXEC_ERROR,
    };

    class BaseMotion
    {
    private:
        friend std::array<std::shared_ptr<BaseMotion>, MOTIONS_COUNT> motions::init_motions();

    protected:
        static void *args;

        BaseMotion() = default;

    public:
        virtual void start() = 0;
        virtual MotionStatus exec() = 0;
        virtual void end() = 0;

        static void set_arguments(void *_args)
        {
            args = _args;
        }
    };

    class ForwardMotion : public BaseMotion
    {
    private:
        uint32_t _l_l_id{0};
        uint32_t _l_r_id{0};
        uint32_t _h_l_id{0};
        uint32_t _h_r_id{0};

        uint32_t iter_counter{0};

#define ACTIONS_COUNT 2

        const static std::array<trajectory::MovingArg_leg_t, ACTIONS_COUNT> _l_l_time_line;
        const static std::array<trajectory::MovingArg_leg_t, ACTIONS_COUNT> _l_r_time_line;
        const static std::array<trajectory::MovingArg_hand_t, ACTIONS_COUNT> _h_l_time_line;
        const static std::array<trajectory::MovingArg_hand_t, ACTIONS_COUNT> _h_r_time_line;

    public:
        // struct Args_t
        // {
        // };

        void start() override;

        MotionStatus exec() override;

        void end() override;
    };

    class FallingForwardMotion : public BaseMotion
    {
    private:
        uint32_t iter_counter{0};

#define FALLING_FRONT_ACTIONS_COUNT 11

        const static std::array<trajectory::MovingArg_leg_t, FALLING_FRONT_ACTIONS_COUNT> _l_l_time_line;
        const static std::array<trajectory::MovingArg_leg_t, FALLING_FRONT_ACTIONS_COUNT> _l_r_time_line;
        const static std::array<trajectory::MovingArg_hand_t, FALLING_FRONT_ACTIONS_COUNT> _h_l_time_line;
        const static std::array<trajectory::MovingArg_hand_t, FALLING_FRONT_ACTIONS_COUNT> _h_r_time_line;

    public:
        // struct Args_t
        // {
        // };

        void start() override;

        MotionStatus exec() override;

        void end() override;
    };

    class FallingBackMotion : public BaseMotion
    {
    private:
        uint32_t iter_counter{0};

#define FALLING_BACK_ACTIONS_COUNT 15

        const static std::array<trajectory::MovingArg_leg_t, FALLING_BACK_ACTIONS_COUNT> _l_l_time_line;
        const static std::array<trajectory::MovingArg_leg_t, FALLING_BACK_ACTIONS_COUNT> _l_r_time_line;
        const static std::array<trajectory::MovingArg_hand_t, FALLING_BACK_ACTIONS_COUNT> _h_l_time_line;
        const static std::array<trajectory::MovingArg_hand_t, FALLING_BACK_ACTIONS_COUNT> _h_r_time_line;

    public:
        // struct Args_t
        // {
        // };

        void start() override;

        MotionStatus exec() override;

        void end() override;
    };

    BaseMotion &get_motion_by_key(MotionKey key);
    uint8_t set_actual_motion(MotionKey key);
    BaseMotion &actual_motion();
    bool has_actual_motion();
    void set_motion_args(void *args);

    void move_to_home_pos(uint32_t time, double acc, double brk);
}