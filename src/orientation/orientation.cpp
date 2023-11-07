#include "orientation.h"
#include "../../sout.h"

#include "IMU.h"
#include "RTOS.h"

#include <array>

namespace orientation
{
    Orient_vector_t::Orient_vector_t() {}
    Orient_vector_t::Orient_vector_t(const double &_x, const double &_y, const double &_z)
        : x(_x), y(_y), z(_z)
    {
    }

    double Orient_vector_t::lenght()
    {
        return sqrt(x * x + y * y + z * z);
    }

    Orient_vector_t Orient_vector_t::operator-(Orient_vector_t const &vec)
    {
        Orient_vector_t res(this->x - vec.x, this->y - vec.y, this->z - vec.z);
        return res;
    }

    cIMU imu;

    Orient_vector_t _rpy_vec, _rpy_vec_last;

    osMutexId _rpy_mutex, _rpy_last_mutex;
    osThreadId _rpy_thread;

#if ORIENT_USE_KALMAN == 1

    struct KalmanFilterParams
    {
        double err_estimate{ORIENT_KALMAN_ERR_MEASURE}, last_estimate{0};
    };

    std::array<KalmanFilterParams, 3> kalman_params;

    double kalman(double new_value, KalmanFilterParams &data)
    {
        double _kalman_gain, _current_estimate;
        _kalman_gain = data.err_estimate / (data.err_estimate + ORIENT_KALMAN_ERR_MEASURE);
        _current_estimate = data.last_estimate + _kalman_gain * (new_value - data.last_estimate);
        data.err_estimate = (1.0 - _kalman_gain) * data.err_estimate + abs(data.last_estimate - _current_estimate) * ORIENT_KALMAN_Q;
        data.last_estimate = _current_estimate;
        return _current_estimate;
    }

#endif

    uint8_t init()
    {
        SERIAL_OUT_L_THRSAFE("Init giro");
        return imu.begin();
    }

    void register_rtos()
    {
        osMutexDef(_rpy_mutex);
        _rpy_mutex = osMutexCreate(osMutex(_rpy_mutex));

        osMutexDef(_rpy_last_mutex);
        _rpy_last_mutex = osMutexCreate(osMutex(_rpy_last_mutex));

        osThreadDef(_rpy_thread, orientation_thread, osPriority::osPriorityNormal, 1, 1024);
        _rpy_thread = osThreadCreate(osThread(_rpy_thread), nullptr);
    }

    void orientation_thread(void const *)
    {
        for (;;)
        {
            update();
            osDelay(ORIENT_UPDATE_PERIOD);
        }
    }

    void update()
    {
        imu.update();

        osMutexWait(_rpy_mutex, portMAX_DELAY);
        osMutexWait(_rpy_last_mutex, portMAX_DELAY);
        _rpy_vec_last = _rpy_vec;
        osMutexRelease(_rpy_last_mutex);
#if ORIENT_USE_KALMAN == 1
        _rpy_vec.x = kalman((double)imu.rpy[0], kalman_params[0]);
        _rpy_vec.y = kalman((double)imu.rpy[1], kalman_params[1]);
        _rpy_vec.z = kalman((double)imu.rpy[2], kalman_params[2]);
#else
        _rpy_vec.x = (double)imu.rpy[0];
        _rpy_vec.y = (double)imu.rpy[1];
        _rpy_vec.z = (double)imu.rpy[2];
#endif
        osMutexRelease(_rpy_mutex);
    }

    void get_rpy(Orient_vector_t &out)
    {
        osMutexWait(_rpy_mutex, portMAX_DELAY);
        out = _rpy_vec;
        osMutexRelease(_rpy_mutex);
    }

    void get_rpy_velocity(Orient_vector_t &out)
    {
        osMutexWait(_rpy_mutex, portMAX_DELAY);
        osMutexWait(_rpy_last_mutex, portMAX_DELAY);
        out = (_rpy_vec - _rpy_vec_last);
        osMutexRelease(_rpy_mutex);
        osMutexRelease(_rpy_last_mutex);
    }

    // Orient_vector_t acc_push_recognize(const Orient_vector_t &acc1, const Orient_vector_t &acc2)
    // {
    //     Orient_vector_t result;
    //     int8_t acc_dir;
    //     if (abs(acc1.x) > ORIENT_ACC_TRESHOLD_VALUE + 750)
    //     {
    //         acc_dir = acc1.x > 0 ? -1 : 1;
    //         if (abs(acc2.x) > 500 && acc2.x * (acc_dir) > 0)
    //         {
    //             result.x = acc1.x;
    //         }
    //     }
    //     if (abs(acc1.y) > ORIENT_ACC_TRESHOLD_VALUE)
    //     {
    //         acc_dir = acc1.y > 0 ? -1 : 1;
    //         if (abs(acc2.y) > 200 && acc2.y * (acc_dir) > 0)
    //         {
    //             result.y = acc1.y;
    //         }
    //     }
    //     if (abs(acc1.z) > ORIENT_ACC_TRESHOLD_VALUE - 750)
    //     {
    //         acc_dir = acc1.z > 0 ? -1 : 1;
    //         if (abs(acc2.z) > 70 && acc2.z * (acc_dir) > 0)
    //         {
    //             result.z = acc1.z;
    //         }
    //     }
    //     return result;
    // }
}
