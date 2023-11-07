// TODO: Сделать лидар блеат
// #pragma once

// #include <inttypes.h>
// #include <vector>

// namespace lidar
// {
//     struct LidarConfig_t
//     {
//         typedef void (*set_lidar_angle_func)(double ang, double speed);
//         typedef uint32_t (*get_lidar_value_func)();

//         uint32_t scanTime{0}; // in milliseconds
//         uint32_t scanResolusion{0}; //count of scan
//         double from_pos{0}; //in rad
//         double to_pos{0};//in rad
//         uint32_t min_input_value{0};
//         uint32_t max_input_value{0};
//         uint32_t treshold_value_rec{0};

//         set_lidar_angle_func &l_a_func;
//         get_lidar_value_func &g_l_func;
//     };

//     struct LidarObject_t
//     {
//         double from_pos{0};
//         double to_pos{0};
//         double avrpos_pos{0};
//     };

//     uint8_t scan(const struct LidarConfig_t &conf);

//     const std::vector<LidarObject_t> &get_objects();
// }