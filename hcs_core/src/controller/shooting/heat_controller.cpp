#include <algorithm>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>

namespace hcs_core::controller::shooting {

// 枪口热量的本地估计（17 mm）：裁判系统报的热量有延迟，所以自己按"每发加热、每拍冷却"推一遍，
// 再算出还能打几发。移植自 RMCS 的 heat_controller.cpp。
//
// 参数：
//   heat_per_shot   每发的热量
//   reserved_heat   留着不用的余量
//
// 冷却量 /referee/shooter/cooling 是**每拍**减掉的热量，由裁判系统那一侧按控制频率换算好。
//
// 每发在规则热量之外多记 10：/gimbal/bullet_fired 是从摩擦轮掉速推出来的，偶尔会漏检，
// 多记一点让估计偏保守。
class HeatController
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    HeatController()
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
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        shooter_heat_ = std::max<std::int64_t>(0, shooter_heat_ - *shooter_cooling_);

        if (*bullet_fired_)
            shooter_heat_ += heat_per_shot_ + kMissedShotMargin;

        *control_bullet_allowance_ = std::max<std::int64_t>(
            0, (*shooter_heat_limit_ - shooter_heat_ - reserved_heat_) / heat_per_shot_);
    }

private:
    static constexpr std::int64_t kMissedShotMargin = 10;

    InputInterface<std::int64_t> shooter_cooling_;
    InputInterface<std::int64_t> shooter_heat_limit_;

    InputInterface<bool> bullet_fired_;

    const std::int64_t heat_per_shot_;
    const std::int64_t reserved_heat_;

    std::int64_t shooter_heat_ = 0;

    OutputInterface<std::int64_t> control_bullet_allowance_;
};

} // namespace hcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::controller::shooting::HeatController, hcs_executor::Component)
