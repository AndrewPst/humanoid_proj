#include "brain.h"

#include "../camera/camera.h"
#include "../orientation/orientation.h"
#include "../dynamixel_drivers/dynamixel.h"

#include "../dynamics/trajectory.h"
#include "../motions/motions.h"

namespace brain
{

    Head_t _head;
    camera::CumObjectMetadata_t _ball_object;
    Ball_t _ball;

    struct
    {
        int8_t x_dir{1};
        int8_t y_dir{1};
    } _ball_find_dir_meta;

    constexpr const uint8_t BALL_CRITICAL_ATTEMPT_REC_COUNT = 30;

    uint8_t aim_to_ball()
    {
        if (_ball.recognized > BALL_CRITICAL_ATTEMPT_REC_COUNT)
        {
            _head.b += 2.0 * DEG_TO_RAD * (double)_ball_find_dir_meta.x_dir;
            if (_head.b <= -M_PI_2 || _head.b >= M_PI_2)
            {
                _head.b = _ball_find_dir_meta.x_dir > 0 ? M_PI_2 : -M_PI_2;
                _ball_find_dir_meta.x_dir *= -1;
                _head.a += 35.0 * DEG_TO_RAD * (double)_ball_find_dir_meta.y_dir;
                if (_head.a <= -5.0 * DEG_TO_RAD || _head.a >= M_PI / 3.0)
                {
                    _head.a = _ball_find_dir_meta.y_dir < 0 ? (M_PI / 3.0) : (-5.0 * DEG_TO_RAD);
                    _ball_find_dir_meta.y_dir *= -1;
                }
            }
            trajectory::move_head(_head.a, _head.b);
        }
        else
        {
            _ball_object = camera::get_object_by_id(0);

            if (_ball_object.metadata_args.cx < (CUM_X_RESOULTION - CUM_X_DEAD_ZONE) / 2 || _ball_object.metadata_args.cx > (CUM_X_RESOULTION + CUM_X_DEAD_ZONE) / 2)
            {
                auto tmp = _head.b;
                if (_ball_object.metadata_args.cx < (CUM_X_RESOULTION - CUM_X_DEAD_ZONE) / 2)
                    tmp += (0.0002 * pow((double)(CUM_X_RESOULTION / 2 - _ball_object.metadata_args.cx), CUM_TRACKING_ACC_COEF));
                else
                    tmp -= (0.0002 * pow((double)(_ball_object.metadata_args.cx - CUM_X_RESOULTION / 2), CUM_TRACKING_ACC_COEF));
                if (tmp > -M_PI_2 && tmp < M_PI_2)
                    _head.b = tmp;
                trajectory::move_head(_head.a, _head.b);
            }
            if (_ball_object.metadata_args.cy < (CUM_Y_RESOULTION - CUM_Y_DEAD_ZONE) / 2 || _ball_object.metadata_args.cy > (CUM_Y_RESOULTION + CUM_Y_DEAD_ZONE) / 2)
            {
                auto tmp = _head.a;
                if (_ball_object.metadata_args.cy < (CUM_Y_RESOULTION - CUM_Y_DEAD_ZONE) / 2)
                    tmp -= (0.00008 * pow((double)(CUM_Y_RESOULTION / 2 - _ball_object.metadata_args.cy), CUM_TRACKING_ACC_COEF));
                else
                    tmp += (0.00008 * pow((double)(_ball_object.metadata_args.cy - CUM_Y_RESOULTION / 2), CUM_TRACKING_ACC_COEF));
                if (tmp > -15.0 * DEG_TO_RAD && tmp < M_PI / 3.0)
                    _head.a = tmp;

                trajectory::move_head(_head.a, _head.b);
            }
        }
        if (camera::object_recognized(0))
        {
            _ball.recognized = 0;
        }
        else
        {
            _ball.recognized = _ball.recognized > BALL_CRITICAL_ATTEMPT_REC_COUNT ? _ball.recognized : _ball.recognized + 1;
            if (_ball.recognized >= BALL_CRITICAL_ATTEMPT_REC_COUNT)
            {
                _ball_find_dir_meta.y_dir = -1;
            }
        }
    }

    orientation::Orient_vector_t _orient_data_absolute;
    uint8_t _orient_falling_counter{0};

    enum GyroResult : uint8_t
    {
        GYRO_OK = 0,
        GYRO_FALLING_BACK,
        GYRO_FALLING_FRONT,
        GYRO_WAKE_UP_BACK,
        GYRO_WAKE_UP_FRONT,
    };

    GyroResult gyro_read()
    {
        orientation::get_rpy(_orient_data_absolute);
#if FALLING_BACK_DIR == 1
        if (ORIENT_INCIDENCE_AXIS > FALLING_BACK_TRESHOLD)
        {
#else
        if (ORIENT_INCIDENCE_AXIS <= FALLING_BACK_TRESHOLD
#if FALLING_ANGLE_OFFSET < 0
            || ORIENT_INCIDENCE_AXIS >= 180 + FALLING_ANGLE_OFFSET
#endif
        )
        {
#endif
            _orient_falling_counter++;
            if (_orient_falling_counter >= 1)
            {
                _orient_falling_counter = 0;
                return GyroResult::GYRO_FALLING_BACK;
            }
        }

#if FALLING_FRONT_DIR == 1
        if (ORIENT_INCIDENCE_AXIS > FALLING_FRONT_TRESHOLD
#if FALLING_ANGLE_OFFSET < 0
            && ORIENT_INCIDENCE_AXIS < 180 + FALLING_ANGLE_OFFSET
#endif
        )
        {
#else
        if (ORIENT_INCIDENCE_AXIS < FALLING_FRONT_TRESHOLD)
        {
#endif
            _orient_falling_counter++;
            if (_orient_falling_counter >= 1)
            {
                _orient_falling_counter = 0;
                return GyroResult::GYRO_FALLING_FRONT;
            }
        }
        return GyroResult::GYRO_OK;
        // SERIAL_BEGIN;
        // SERIAL_OUT(ORIENT_INCIDENCE_AXIS);
        // SERIAL_OUT('\t')
        // SERIAL_OUT(vec.y);
        // SERIAL_OUT('\t')
        // SERIAL_OUT(ORIENT_COMPASS_AXIS);
        // SERIAL_OUT_L('\t');
        // SERIAL_END;
    }

    void wait_motion_ended()
    {
        if (motions::has_actual_motion() == false)
            return;
        motions::MotionStatus st{motions::MotionStatus::M_STATUS_IN_EXECUTION};
        while (st == motions::MotionStatus::M_STATUS_IN_EXECUTION)
        {
            st = motions::actual_motion().exec();
            osDelay(2);
        }
        if (st == motions::MotionStatus::M_STATUS_EXEC_ERROR)
        {
            SERIAL_OUT_L_THRSAFE("[BRAIN] motion wait err");
        }
        SERIAL_OUT_L_THRSAFE("[BRAIN] Wait end");
        motions::set_actual_motion(motions::MotionKey::MOTION_UNKNOWN);
    }

    bool _head_enable{true};
    uint32_t _last_head_move_time{0};

    bool _gyro_enable{true};
    uint32_t _last_gyro_read_time{0};

    void main()
    {
        orientation::Orient_vector_t _orient_data_absolute;
        for (;;)
        {
            if (millis() - _last_head_move_time >= 1000 / CUM_FPS / 2.0 && _head_enable)
            {
                aim_to_ball();
                _last_head_move_time = millis();
            }
            if (millis() - _last_gyro_read_time > 5 && _gyro_enable)
            {
                auto _gyro_res = gyro_read();
                if (_gyro_res == GyroResult::GYRO_FALLING_BACK)
                {
                    motions::set_actual_motion(motions::MotionKey::MOTION_FALLING_BACK);
                    wait_motion_ended();
                }
                else if (_gyro_res == GyroResult::GYRO_FALLING_FRONT)
                {
                    motions::set_actual_motion(motions::MotionKey::MOTION_FALLING_FRONT);
                    wait_motion_ended();
                }
                _last_gyro_read_time = millis();
            }
            if (motions::has_actual_motion())
            {
                auto st = motions::actual_motion().exec();
                if (st == motions::MotionStatus::M_STATUS_EXEC_ERROR)
                {
                    SERIAL_OUT_L_THRSAFE("[BRAIN] motion err");
                }
            }

            osDelay(2);
        }
    }
} // namespace brain
