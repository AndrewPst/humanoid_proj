#include "camera.h"

#include <array>
#include <inttypes.h>
#include "../../config.h"
#include "../../construction.h"
#include <RTOS.h>

namespace camera
{

    get_camera_data_func _camera_data_func;

    osMutexId _data_buf_mutex{nullptr};
    osThreadId _thread_id{nullptr};

    std::array<CumObjectMetadata_t, CUM_OBJECTS_COUNT> _objects;

#if CUM_USE_KALMAN == 1
    struct KalmanFilterParams
    {
        double err_estimate{CAM_KALMAN_ERR_MEASURE}, last_estimate{0};
    };

    std::array<KalmanFilterParams, CAM_USE_KALMAN_FOR_CX + CAM_USE_KALMAN_FOR_CY + CAM_USE_KALMAN_FOR_AREA> kalman_params;

    double kalman(double new_value, KalmanFilterParams &data)
    {
        double _kalman_gain, _current_estimate;
        _kalman_gain = data.err_estimate / (data.err_estimate + CAM_KALMAN_ERR_MEASURE);
        _current_estimate = data.last_estimate + _kalman_gain * (new_value - data.last_estimate);
        data.err_estimate = (1.0 - _kalman_gain) * data.err_estimate + abs(data.last_estimate - _current_estimate) * CAM_KALMAN_Q;
        data.last_estimate = _current_estimate;
        return _current_estimate;
    }

#endif

    void camera_register_rtos()
    {
        osMutexDef(_data_buf_mutex);
        _data_buf_mutex = osMutexCreate(osMutex(_data_buf_mutex));

        osThreadDef(_thread_id, thread_camera, osPriority::osPriorityNormal, 1, 1024);
        _thread_id = osThreadCreate(osThread(_thread_id), NULL);
    }

    void thread_camera(const void *)
    {
        for (;;)
        {
            for (uint8_t i = 0; i < CUM_OBJECTS_COUNT; ++i)
            {
                osMutexWait(_data_buf_mutex, portMAX_DELAY);
                _camera_data_func(i, _objects[i].buffer.data());
                osMutexRelease(_data_buf_mutex);
            }
            osDelay(CUM_UPDATE_PERIOD_MS); // Блокирует только этот поток. Не заменяйте на всякие уебанские таймеры
        }
    }

    void set_cameraDataGetterFunc(const get_camera_data_func &func)
    {
        _camera_data_func = func;
    }

    const CumObjectMetadata_t &get_object_by_id(uint8_t id)
    {
        return _objects[id];
    }

    void lock_data_mutex()
    {
        osMutexWait(_data_buf_mutex, portMAX_DELAY);
    }

    void unlock_data_mutex()
    {
        osMutexRelease(_data_buf_mutex);
    }

    void filter_data(CumObjectMetadata_t &data)
    {
#if CAM_USE_KALMAN_FOR_CX == 1
        data.metadata_args.cx = (uint16_t)kalman((double)data.metadata_args.cx, kalman_params[0]);
#endif
#if CAM_USE_KALMAN_FOR_CY == 1
        data.metadata_args.cy = (uint16_t)kalman((double)data.metadata_args.cy, kalman_params[1]);
#endif
#if CAM_USE_KALMAN_FOR_AREA == 1
        data.metadata_args.area = (uint16_t)kalman((double)data.metadata_args.area, kalman_params[2]);
#endif
    }
}