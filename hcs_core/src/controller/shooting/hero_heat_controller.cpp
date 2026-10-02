#include <algorithm>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>

namespace hcs_core::controller::shooting {

// 枪口热量的本地估计（42 mm，英雄）。和 HeatController 的区别：
//   - 发射只在 /gimbal/bullet_fired 的上升沿记一次（英雄的发射信号会保持好几拍）；
//   - 不多记余量；
//   - 把估计的热量也发布出来（/shoot/heat），给界面看。
// 移植自 RMCS 的 hero_heat_controller.cpp。
//
// 参数：
//   heat_per_shot   每发的热量
//   reserved_heat   留着不用的余量
class HeroHeatController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    HeroHeatController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , heat_per_shot_(get_parameter("heat_per_shot").as_int())
        , reserved_heat_(get_parameter("reserved_heat").as_int()) {

        register_input("/referee/shooter/cooling", shooter_cooling_);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_);

        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_output(
            "/gimbal/control_bullet_allowance/limited_by_heat", control_bullet_allowance_, 0);
        register_output("/shoot/heat", shooting_heat_, 0.0);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        shooter_heat_ = std::max<std::int64_t>(0, shooter_heat_ - *shooter_cooling_);

        const bool bullet_fired = *bullet_fired_;
        if (bullet_fired && !last_bullet_fired_)
            shooter_heat_ += heat_per_shot_;
        last_bullet_fired_ = bullet_fired;

        *control_bullet_allowance_ = std::max<std::int64_t>(
            0, (*shooter_heat_limit_ - shooter_heat_ - reserved_heat_) / heat_per_shot_);

        *shooting_heat_ = static_cast<double>(shooter_heat_);
    }

private:
    InputInterface<std::int64_t> shooter_cooling_;
    InputInterface<std::int64_t> shooter_heat_limit_;

    InputInterface<bool> bullet_fired_;

    const std::int64_t heat_per_shot_;
    const std::int64_t reserved_heat_;

    bool last_bullet_fired_ = false;
    std::int64_t shooter_heat_ = 0;
    OutputInterface<double> shooting_heat_;

    OutputInterface<std::int64_t> control_bullet_allowance_;
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    hcs_core::controller::shooting::HeroHeatController, hcs_executor::Component)
