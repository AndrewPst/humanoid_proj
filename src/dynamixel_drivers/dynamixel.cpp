#include "dynamixel.h"

#include "../../sout.h"
#include "../../config.h"
#include "../../construction.h"

namespace dynamixel
{

    const std::array<double, SERVO_COUNT> _servo_backlash_comp_arr{
        LEG_LEFT_BACKLASH_COMP_0_SERVO,
        LEG_LEFT_BACKLASH_COMP_1_SERVO,
        LEG_LEFT_BACKLASH_COMP_2_SERVO,
        LEG_LEFT_BACKLASH_COMP_3_SERVO,
        LEG_LEFT_BACKLASH_COMP_4_SERVO,
        LEG_LEFT_BACKLASH_COMP_5_SERVO,
        LEG_RIGHT_BACKLASH_COMP_0_SERVO,
        LEG_RIGHT_BACKLASH_COMP_1_SERVO,
        LEG_RIGHT_BACKLASH_COMP_2_SERVO,
        LEG_RIGHT_BACKLASH_COMP_3_SERVO,
        LEG_RIGHT_BACKLASH_COMP_4_SERVO,
        LEG_RIGHT_BACKLASH_COMP_5_SERVO,
        HAND_LEFT_BACKLASH_COMP_0_SERVO,
        HAND_LEFT_BACKLASH_COMP_1_SERVO,
        HAND_LEFT_BACKLASH_COMP_2_SERVO,
        HAND_RIGHT_BACKLASH_COMP_0_SERVO,
        HAND_RIGHT_BACKLASH_COMP_1_SERVO,
        HAND_RIGHT_BACKLASH_COMP_2_SERVO,
        HEAD_BACKLASH_COMP_0_SERVO,
        HEAD_BACKLASH_COMP_1_SERVO,
    };

    DynamixelWorkbench dxl;

    osMutexId _dxl_mutex;

    void dynamixel_register_rtos()
    {
        osMutexDef(_dxl_mutex);
        _dxl_mutex = osMutexCreate(osMutex(_dxl_mutex));
    }

    uint8_t init(const struct Dynamixel_config_t &config)
    {
        const char *log;
        osMutexWait(_dxl_mutex, portMAX_DELAY);
        uint8_t result = dxl.init(config.device_name, config.boudrate, &log);
        osMutexRelease(_dxl_mutex);
        SERIAL_BEGIN
        SERIAL_OUT("[DYNAMIXEL] Init result:\t");
        SERIAL_OUT(result);
        if (result == false)
        {
            SERIAL_OUT('\t');
            SERIAL_OUT(log);
        }
        SERIAL_OUT_L();
        SERIAL_END;
        return !result;
    }

    uint8_t findConnections(std::vector<uint8_t> *out, uint8_t max_servo_count)
    {
        bool result{0};
        uint16_t model_number{0};
        const char *log{nullptr};
        uint8_t s_count{0};
        uint8_t last_succ_id{0};
        if (out)
            out->clear();
        for (uint8_t d_id = 1; d_id < 255; d_id++)
        {
            if (s_count >= max_servo_count)
                break;
            osMutexWait(_dxl_mutex, portMAX_DELAY);
            result = dxl.ping(d_id, &model_number, &log);
            osMutexRelease(_dxl_mutex);
            if (result == false)
            {
                // SERIAL_BEGIN
                // SERIAL_OUT("Failed to ping:\t");
                // SERIAL_OUT_L(log);
                // SERIAL_END
            }
            else
            {
                SERIAL_BEGIN
                SERIAL_OUT("Succeeded to ping:\t");
                SERIAL_OUT("id : ");
                SERIAL_OUT(d_id);
                SERIAL_OUT("\tmodel_number : ");
                SERIAL_OUT_L(model_number);
                SERIAL_END

                if (out)
                    out->push_back(d_id);
                s_count++;
                last_succ_id = d_id;
            }
            osDelay(10);
        }
        if (s_count)
        {
            osMutexWait(_dxl_mutex, portMAX_DELAY);
            dxl.ping(last_succ_id, &model_number);
            osMutexRelease(_dxl_mutex);
        }
        return s_count;
    }

    uint8_t checkConnections(const std::vector<uint8_t> &ids)
    {
        bool result{0};
        uint16_t model_number{0};
        const char *log{nullptr};
        uint8_t s_count{0};
        uint8_t last_succ_id{0};
        for (uint8_t d_id = 0; d_id < ids.size(); d_id++)
        {
            osMutexWait(_dxl_mutex, portMAX_DELAY);
            result = dxl.ping(ids[d_id], &model_number, &log);
            osMutexRelease(_dxl_mutex);
            if (result == false)
            {
                // SERIAL_BEGIN
                // SERIAL_OUT("Failed to ping:\t");
                // SERIAL_OUT_L(log);
                // SERIAL_END
            }
            else
            {
                SERIAL_BEGIN
                SERIAL_OUT("Succeeded to ping:\t");
                SERIAL_OUT("id : ");
                SERIAL_OUT(ids[d_id]);
                SERIAL_OUT("\tmodel_number : ");
                SERIAL_OUT_L(model_number);
                SERIAL_END

                s_count++;
                last_succ_id = ids[d_id];
            }
            osDelay(10);
        }
        if (s_count)
        {
            osMutexWait(_dxl_mutex, portMAX_DELAY);
            dxl.ping(last_succ_id, &model_number);
            osMutexRelease(_dxl_mutex);
        }
        return s_count;
    }

    uint8_t checkCamera()
    {
        const char *log{nullptr};
        osMutexWait(_dxl_mutex, portMAX_DELAY);
        auto result = dxl.ping(CAM_ID, &log);
        osMutexRelease(_dxl_mutex);
        if (result == false)
            SERIAL_OUT_L_THRSAFE(log);
        return !result;
    }

    uint8_t setupDrivers(const std::vector<uint8_t> &ids, int32_t default_velocity)
    {
        bool result{false};
        const char *log;
        uint8_t succ_count{0};
        for (auto i : ids)
        {
            osMutexWait(_dxl_mutex, portMAX_DELAY);
            result = dxl.jointMode(i, default_velocity, 0, &log);
            osMutexRelease(_dxl_mutex);
            if (result)
            {
                // SERIAL_BEGIN;
                // SERIAL_OUT("Succeeded to setup servo\tID: ");
                // SERIAL_OUT_L(i);
                // SERIAL_END;
                succ_count++;
            }
            else
            {
                SERIAL_BEGIN;
                SERIAL_OUT("Failed to setup servo\tID: ");
                SERIAL_OUT(i);
                SERIAL_OUT("\tLOG: ");
                SERIAL_OUT_L(log);
                SERIAL_END;
            }
            osDelay(10);
        }
        if (succ_count)
        {
            osMutexWait(_dxl_mutex, portMAX_DELAY);
            result = dxl.addSyncWriteHandler(ids[0], "Goal_Position", &log);
            if (result == false)
                SERIAL_OUT_L_THRSAFE(log);
            result = dxl.addSyncWriteHandler(ids[0], "Moving_Speed", &log);
            if (result == false)
                SERIAL_OUT_L_THRSAFE(log);
            result = dxl.addSyncReadHandler(ids[0], "Present_Position", &log);
            osMutexRelease(_dxl_mutex);
            if (result == false)
                SERIAL_OUT_L_THRSAFE(log);
        }
        return succ_count;
    };

    uint8_t syncWrite(const uint8_t *ids, const int32_t *data, uint8_t count, SyncWriteParamType paramType)
    {
        const char *log;
        osMutexWait(_dxl_mutex, portMAX_DELAY);
        auto r = dxl.syncWrite(paramType, (uint8_t *)ids, count, (int32_t *)data, 1, &log);
        osMutexRelease(_dxl_mutex);
        // if (r == false)
        //     SERIAL_OUT_L_THRSAFE(log);
        return !r;
    }

    uint8_t syncWrite(const std::vector<uint8_t> &ids, const std::vector<int32_t> &data, SyncWriteParamType paramType)
    {
        if (data.size() < ids.size())
            return 1;
        return syncWrite(ids.data(), data.data(), ids.size(), paramType);
    }

    uint8_t syncReadPosition(const uint8_t *ids, int32_t *out, uint8_t count)
    {
        const char *log{nullptr};
        bool result{false};
        uint8_t ret{0};
        for (uint8_t i = 0; i < count; i++)
        {
            osMutexWait(_dxl_mutex, portMAX_DELAY);
            result = dxl.getPresentPositionData(ids[i], out + i, &log);
            osMutexRelease(_dxl_mutex);
            if (result)
                ret++;
            else
                SERIAL_OUT_L_THRSAFE(log);
        }
        return ret;
    }

    uint8_t syncReadPosition(const std::vector<uint8_t> &ids, std::vector<int32_t> &out)
    {
        out.resize(ids.size());
        return syncReadPosition(ids.data(), out.data(), ids.size());
    }

    uint8_t rad_to_value_arr(const double *in, int32_t *out, uint8_t count,
                             const double *offsets, int8_t offset_factor)
    {
        uint8_t i{0};
        for (i = 0; i < count; ++i)
        {
            out[i] = RAD2VALUE(in[i] + (offsets ? offsets[i] * (double)offset_factor : 0));
        }
        return i;
    }

    uint8_t value_to_rad_arr(const int32_t *in, double *out, uint8_t count,
                             const double *offsets, int8_t offset_factor)
    {
        uint8_t i{0};
        for (i = 0; i < count; ++i)
        {
            out[i] = VALUE2RAD(in[i]) - (offsets ? offsets[i] * (double)offset_factor : 0);
        }
        return i;
    }

    double _divFunc(const double &v1, const double v2)
    {
        return v1 / v2;
    }

    double _multFunc(const double &v1, const double v2)
    {
        return v1 * v2;
    }

    typedef double (*operFunc_t)(const double &v1, const double v2);

    uint8_t values_backlash_compensate_rad(const double *in, double *out, uint8_t size, const double *backlash_com_arr, uint8_t mode)
    {
        operFunc_t f;
        if (mode)
            f = _multFunc;
        else
            f = _divFunc;
        uint8_t i{0};
        for (; i < size; ++i)
        {
            out[i] = f(in[i], backlash_com_arr[i]);
        }
        return i;
    }

    uint8_t readCumRegister(uint16_t address, uint8_t *out)
    {
        const char *log;
        osMutexWait(_dxl_mutex, portMAX_DELAY);
        if (!dxl.readRegisterBytes(CAM_ID, CUM_DATA_OFFSET_BYTES + address * CUM_DATA_SIZE_BYTES, CUM_DATA_SIZE_BYTES, out, &log))
        {
            osMutexRelease(_dxl_mutex);
            SERIAL_OUT_L_THRSAFE(log);
            return 1;
        }
        osMutexRelease(_dxl_mutex);
        return 0;
    }

    const std::array<double, SERVO_COUNT> &servo_backlash_comp_buffer()
    {
        return _servo_backlash_comp_arr;
    }
};