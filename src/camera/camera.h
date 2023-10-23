#pragma once

#include <inttypes.h>
#include <array>

namespace camera
{
    typedef uint8_t (*get_camera_data_func)(uint16_t obj_addres, uint8_t *out);

    union CumObjectMetadata_t
    {
        std::array<uint8_t, 16> buffer;
        struct MetadataNamed_t
        {
            uint8_t type;
            uint8_t dummy;
            uint16_t cx;
            uint16_t cy;
            uint16_t area; // было 32
            uint16_t left;
            uint16_t right;
            uint16_t top;
            uint16_t bottom;
        } metadata_args;
    };

    void camera_register_rtos();

    void thread_camera(const void *);

    void set_cameraDataGetterFunc(const get_camera_data_func &func);

    const CumObjectMetadata_t &get_object_by_id(uint8_t id);
    void filter_data(CumObjectMetadata_t& data);

    void lock_data_mutex();
    void unlock_data_mutex();
}