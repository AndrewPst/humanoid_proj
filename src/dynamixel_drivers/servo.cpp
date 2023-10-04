#include "servo.h"

#include "../../sout.h"

namespace dynamixel
{

    osThreadId thread_id_dynamixel;
    osMutexId _mut_arr_pos_buf;

    void dynamixel_register_rtos()
    {
        osMutexDef(_mut_arr_pos_buf);
        osMutexCreate(osMutex(_mut_arr_pos_buf));

        osThreadDef(THREAD_DYNAMIXEL, dynamixel::thread_dynamixel, osPriorityNormal, 0, 8192);
        thread_id_dynamixel = osThreadCreate(osThread(THREAD_DYNAMIXEL), NULL);
    }

    void thread_dynamixel(void const *arg)
    {
        (void)arg;
        for (;;)
        {
            // TODO read commands queue;

            osDelay(500);
            SERIAL_OUT_L_THRSAFE("Work!");
        }
    }

    uint8_t init(const struct Dynamixel_config_t &config)
    {
        SERIAL_OUT_L_THRSAFE("Start init");
        const char *log;
        uint8_t result = dxl.init(config.device_name, config.boudrate, &log);
        SERIAL_BEGIN
        SERIAL_OUT("Init result:\t");
        SERIAL_OUT(result);
        SERIAL_OUT('\t');
        SERIAL_OUT(log);
        SERIAL_OUT_L(dxl.getProtocolVersion());
        SERIAL_END;
        return result;
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
        for (uint8_t d_id = 0; d_id < 255; d_id++)
        {
            if (s_count >= max_servo_count)
                break;
            result = dxl.ping(d_id, &model_number, &log);
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
            dxl.ping(last_succ_id, &model_number);
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
            result = dxl.ping(ids[d_id], &model_number, &log);
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
            dxl.ping(last_succ_id, &model_number);
        }
        return s_count;
    }

    uint8_t setupDrivers(const std::vector<uint8_t> &ids, int32_t default_velocity, int32_t default_acc)
    {
        bool result{false};
        const char *log;
        uint8_t succ_count{0};
        for (auto i : ids)
        {
            result = dxl.jointMode(i, default_velocity, default_acc, &log);
            if (result)
            {
                SERIAL_BEGIN;
                SERIAL_OUT("Succeeded to setup servo\tID: ");
                SERIAL_OUT_L(i);
                SERIAL_END;
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
            dxl.addSyncWriteHandler(ids[0], "Goal_Position");
            dxl.addSyncWriteHandler(ids[0], "Moving_Speed");

            result = dxl.addSyncReadHandler(ids[0], "Present_Position", &log);
            SERIAL_OUT_L_THRSAFE(log);
        }
        return succ_count;
    };

    uint8_t syncWrite(const uint8_t *ids, const int32_t *data, uint8_t count, SyncWriteParamType paramType)
    {
        const char *log;
        auto r = dxl.syncWrite(paramType, (uint8_t *)ids, count, (int32_t *)data, 1, &log);
        SERIAL_OUT_L_THRSAFE(log);
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
            result = dxl.getPresentPositionData(ids[i], out + i, &log);
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

    uint8_t rad_to_value_arr(const double *in, int32_t *out, uint8_t count, const double *offsets, int8_t offset_factor)
    {
        uint8_t i{0};
        for (i = 0; i < count; ++i)
        {
            out[i] = RAD2VALUE(in[i] + (offsets ? offsets[i] * (double)offset_factor : 0));
        }
        return i;
    }

};