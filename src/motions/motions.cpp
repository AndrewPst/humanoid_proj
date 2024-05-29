#include "motions.h"

#include "../../motion_setup.h"

namespace motions
{
    void *BaseMotion::args;

    std::array<std::shared_ptr<BaseMotion>, MOTIONS_COUNT> init_motions()
    {
        return {{
            std::shared_ptr<BaseMotion>(new ForwardMotion()),
            std::shared_ptr<BaseMotion>(new FallingBackMotion()),
            std::shared_ptr<BaseMotion>(new FallingForwardMotion()),
        }};
    }

    std::array<std::shared_ptr<BaseMotion>, MOTIONS_COUNT> motions = init_motions();

    BaseMotion &get_motion_by_key(MotionKey key)
    {
        return *motions[key];
    }

    std::shared_ptr<BaseMotion> _actual_motion{nullptr};

    uint8_t set_actual_motion(MotionKey key)
    {
        if (has_actual_motion())
            actual_motion().end();
        if (key == MotionKey::MOTION_UNKNOWN)
        {
            _actual_motion = nullptr;
        }
        else
        {
            _actual_motion = motions[key];
            _actual_motion->start();
        }
        return 0;
    }

    BaseMotion &actual_motion()
    {
        return *_actual_motion;
    }

    bool has_actual_motion()
    {
        return _actual_motion.get();
    }

    void set_motion_args(void *args)
    {
        BaseMotion::set_arguments(args);
    }

    void move_to_home_pos(uint32_t time, double acc, double brk)
    {
        trajectory::set_new_goal_pos_l_l({trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_l_pos), time, acc, brk});
        trajectory::set_new_goal_pos_l_r({trajectory::make_pos_arr<kinematics::pos_dec_t>(_default_l_r_pos), time, acc, brk});
        trajectory::set_new_goal_pos_h_l({trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_l_pos), time, acc, brk});
        trajectory::set_new_goal_pos_h_r({trajectory::make_pos_arr<kinematics::pos_cylindrical_t>(_default_h_r_pos), time, acc, brk});
        trajectory::wait_all_executed();
    }
}