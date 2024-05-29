#pragma once

#include "../../config.h"
#include <inttypes.h>

namespace orientation
{
    struct Orient_vector_t
    {
        double x{0};
        double y{0};
        double z{0};

        Orient_vector_t();
        Orient_vector_t(const double &x, const double &y, const double &z);

        double lenght();

        Orient_vector_t operator-(Orient_vector_t const &vec);
    };

    uint8_t init();

    void register_rtos();

    void update();

    void orientation_thread(void const *);

    void get_rpy(Orient_vector_t &out);
    void get_rpy_velocity(Orient_vector_t &out);

    // Orient_vector_t acc_push_recognize(const Orient_vector_t &acc1, const Orient_vector_t &acc2);
}
