#pragma once

#include <inttypes.h>

namespace motion
{

    enum MotionStatus : uint8_t
    {
        MOTION_STATUS_EXECUTION = 0,
        MOTION_STATUS_ENDED,
        MOTION_STATUS_ERROR,
    };

    namespace ForwardMoving
    {
        void pre_init();
        void on_start();
        MotionStatus execute();
        void on_end();
    }

}