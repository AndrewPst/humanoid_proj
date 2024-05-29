#pragma once

#include <Arduino.h>
#include <inttypes.h>

namespace brain
{

    struct Ball_t
    {
        int32_t distance{0};
        uint8_t recognized{0};
    };

    struct Head_t
    {
        double a{0};
        double b{0};
    };

    void main();

}