#include <eigen3/Eigen/Geometry>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_description/tf_description.hpp>
#include <hcs_executor/component.hpp>

namespace hcs_core::controller::gimbal {

// 两轴云台的运动学树：yaw / pitch 关节角与云台 IMU 姿态 → /tf。
// 原来长在整车硬件组件里；它是运动学，不是硬件，所以单独成组件，硬件组件只出
// 各设备自己的量。执行器按依赖排序，本组件排在硬件之后、用 /tf 的控制器之前。
class GimbalTf : public hcs_executor::Component {
public:
    GimbalTf() {
        register_input("/gimbal/yaw/angle", yaw_angle_);
        register_input("/gimbal/pitch/angle", pitch_angle_);
        register_input("/gimbal/imu/quaternion", imu_quaternion_);

        register_output("/tf", tf_);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        tf_->set_state<hcs_description::GimbalCenterLink, hcs_description::YawLink>(*yaw_angle_);
        tf_->set_state<hcs_description::YawLink, hcs_description::PitchLink>(*pitch_angle_);
        tf_->set_transform<hcs_description::PitchLink, hcs_description::OdomImu>(*imu_quaternion_);
    }

private:
    InputInterface<double> yaw_angle_;
    InputInterface<double> pitch_angle_;
    InputInterface<Eigen::Quaterniond> imu_quaternion_;

    OutputInterface<hcs_description::Tf> tf_;
};

} // namespace hcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::controller::gimbal::GimbalTf, hcs_executor::Component)
