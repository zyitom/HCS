#include <cstdint>
#include <cstring>
#include <eigen3/Eigen/Eigen>
#include <rclcpp/node.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_msgs/game_stage.hpp>
#include <hcs_msgs/robot_id.hpp>
#include <hcs_msgs/serial_interface.hpp>
#include <hcs_base/protocol/dji_crc.hpp>
#include <hcs_base/protocol/package_receive.hpp>
#include <hcs_base/tick_timer.hpp>

#include "referee/frame.hpp"
#include "referee/status/field.hpp"

namespace hcs_core::referee {
using namespace status;

class Status
    : public hcs_executor::Component
    , public rclcpp::Node {
public:
    Status()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/referee/serial", serial_);

        register_output("/referee/game/stage", game_stage_, hcs_msgs::GameStage::UNKNOWN);
        register_output("/referee/game/stage_remain_time", stage_remain_time_, 0);
        register_output("/referee/game/sync_timestamp", sync_timestamp_, uint64_t{0});
        register_output(
            "/referee/event/ally_big_energy_activation_status", ally_big_energy_activation_status_,
            0);
        register_output(
            "/referee/event/ally_small_energy_activation_status",
            ally_small_energy_activation_status_, 0);
        register_output(
            "/referee/event/ally_fortress_occupation_status", ally_fortress_occupation_status_, 0);
        register_output(
            "/referee/dart/latest_hit_target_total_count", dart_latest_hit_target_total_count_, 0);

        register_output("/referee/id", robot_id_, hcs_msgs::RobotId::UNKNOWN);
        register_output("/referee/shooter/cooling", robot_shooter_cooling_, 0);
        register_output("/referee/shooter/heat_limit", robot_shooter_heat_limit_, 0);
        register_output("/referee/chassis/power_limit", robot_chassis_power_limit_, 0.0);
        register_output("/referee/chassis/power", robot_chassis_power_, 0.0);
        register_output("/referee/chassis/buffer_energy", robot_buffer_energy_, 60.0);
        register_output("/referee/chassis/output_status", chassis_output_status_, false);

        register_output("/referee/sentry/posture", sentry_posture_, uint8_t{3});
        register_output("/referee/sentry/is_powered", sentry_is_powered_, false);
        register_output("/referee/sentry/is_disengaged", sentry_is_disengaged_, false);
        register_output("/referee/sentry/can_rebirth_free", sentry_can_rebirth_free_, false);
        register_output("/referee/sentry/can_rebirth_gold", sentry_can_rebirth_gold_, false);
        register_output(
            "/referee/sentry/rebirth_gold_cost", sentry_rebirth_gold_cost_, uint16_t{0});

        register_output("/referee/robots/hp", robots_hp_);
        register_output("/referee/ally/hero_hp", ally_hero_hp_, 0);
        register_output("/referee/ally/engineer_hp", ally_engineer_hp_, 0);
        register_output("/referee/ally/infantry_1_hp", ally_infantry_1_hp_, 0);
        register_output("/referee/ally/infantry_2_hp", ally_infantry_2_hp_, 0);
        register_output("/referee/ally/outpost/hp", ally_outpost_hp_, 0);
        register_output("/referee/ally/base/hp", ally_base_hp_, 0);
        register_output("/referee/enemy/outpost/hp", enemy_outpost_hp_, 0);
        register_output("/referee/enemy/base/hp", enemy_base_hp_, 0);
        register_output("/referee/damage_difference", damage_difference_, int16_t{0});
        register_output("/referee/current_hp", robot_current_hp_);
        register_output("/referee/position/x", robot_position_x_, 0.0);
        register_output("/referee/position/y", robot_position_y_, 0.0);
        register_output("/referee/position/angle", robot_position_angle_, 0.0);
        register_output("/referee/shooter/bullet_allowance", robot_bullet_allowance_, false);
        register_output(
            "/referee/shooter/42mm_bullet_allowance", robot_42mm_bullet_allowance_, false);
        register_output(
            "/referee/shooter/fortress_17mm_bullet_allowance",
            robot_fortress_17mm_bullet_allowance_, 0);
        register_output("/referee/remaining_gold_coin", remaining_gold_coin_, 0);

        register_output("/referee/shooter/initial_speed", robot_initial_speed_, false);
        register_output("/referee/shooter/shoot_timestamp", robot_shoot_timestamp_, false);

        register_output(
            "/referee/map_command/target_position_x", map_command_target_position_x_, 0.0);
        register_output(
            "/referee/map_command/target_position_y", map_command_target_position_y_, 0.0);
        register_output("/referee/map_command/keyboard", map_command_keyboard_, 0);
        register_output("/referee/map_command/target_robot_id", map_command_target_robot_id_, 0);
        register_output("/referee/map_command/source", map_command_source_, 0);
        register_output(
            "/referee/map_command/received_timestamp", map_command_received_timestamp_, 0.0);
        register_output(
            "/referee/map_command/event/target_position_x", map_command_event_target_position_x_,
            0.0);
        register_output(
            "/referee/map_command/event/target_position_y", map_command_event_target_position_y_,
            0.0);
        register_output("/referee/map_command/event/keyboard", map_command_event_keyboard_, 0);
        register_output(
            "/referee/map_command/event/target_robot_id", map_command_event_target_robot_id_, 0);
        register_output("/referee/map_command/event/source", map_command_event_source_, 0);
        register_output("/referee/map_command/event/timestamp", map_command_event_timestamp_, 0.0);
        register_output("/referee/map_command/event/sequence", map_command_event_sequence_, 0);

        robot_status_watchdog_.reset(5'000);
    }

    void update(const hcs_sync::Tick&) HCS_NONBLOCKING override {
        if (!serial_.active())
            return;

        if (cache_size_ >= sizeof(frame_.header)) {
            auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id)
                            + frame_.header.data_length + sizeof(uint16_t);
            // data_length 来自线上,CRC8 只保完整性不保合法性:16 位的长度字段
            // 最大 65535,而 frame_ 是定长的。不钳制,一个 CRC 恰好合法的超长帧
            // 就会让 read 直接写出 frame_ 的边界。
            if (frame_size > sizeof(frame_)) {
                if (const auto count = oversized_frames_.hit())
                    logger().rt().warn(
                        "Frame data_length {} out of range, dropped ({} so far)",
                        static_cast<unsigned>(frame_.header.data_length), *count);
                cache_size_ = 0;
            } else {
                cache_size_ += serial_->read(
                    reinterpret_cast<std::byte*>(&frame_) + cache_size_,
                    frame_size - cache_size_);

                if (cache_size_ == frame_size) {
                    cache_size_ = 0;
                    if (hcs_utility::dji_crc::verify_crc16(&frame_, frame_size)) {
                        process_frame();
                    } else if (const auto count = body_crc_errors_.hit()) {
                        logger().rt().warn("Body crc16 invalid ({} so far)", *count);
                    }
                }
            }
        } else {
            auto result = hcs_utility::receive_package<std::byte>(
                const_cast<hcs_msgs::SerialInterface&>(*serial_), frame_.header, cache_size_,
                static_cast<uint8_t>(0xa5), [](const FrameHeader& header) {
                    return hcs_utility::dji_crc::verify_crc8(header);
                });
            if (result == hcs_utility::ReceiveResult::HEADER_INVALID) {
                if (const auto count = header_start_errors_.hit())
                    logger().rt().warn("Header start invalid ({} so far)", *count);
            } else if (result == hcs_utility::ReceiveResult::VERIFY_INVALID) {
                if (const auto count = header_crc_errors_.hit())
                    logger().rt().warn("Header crc8 invalid ({} so far)", *count);
            }
        }

        if (game_status_watchdog_.tick()) {
            logger().rt().info("Game status receiving timeout. Set stage to unknown.");
            *game_stage_ = hcs_msgs::GameStage::UNKNOWN;
        }
        if (robot_status_watchdog_.tick()) {
            logger().rt().error("Robot status receiving timeout. Set to safe indicators.");
            *robot_shooter_cooling_ = safe_shooter_cooling;
            *robot_shooter_heat_limit_ = safe_shooter_heat_limit;
            *robot_chassis_power_limit_ = safe_chassis_power_limit;
        }
        if (power_heat_data_watchdog_.tick()) {
            logger().rt().error("Power heat data receiving timeout. Set to initial values.");
            *robot_chassis_power_ = 0.0;
            *robot_buffer_energy_ = 60.0;
        }
    }

private:
    /// 帧体按具体消息类型取出。wire 结构对小端主机的依赖见 field.hpp 顶部;
    /// 类型自身的长度由各 static_assert(sizeof) 钉死,帧长合法性由收帧路径保证。
    template <typename T>
    T& frame_body_as() {
        return reinterpret_cast<T&>(frame_.body.data);
    }

    void process_frame() {
        auto command_id = frame_.body.command_id;
        if (command_id == 0x0001)
            update_game_status();
        else if (command_id == 0x0003)
            update_game_robot_hp();
        else if (command_id == 0x0101)
            update_event_data();
        else if (command_id == 0x0105)
            update_dart_info();
        else if (command_id == 0x0201)
            update_robot_status();
        else if (command_id == 0x0202)
            update_power_heat_data();
        else if (command_id == 0x0203)
            update_robot_position();
        else if (command_id == 0x0206)
            update_hurt_data();
        else if (command_id == 0x0207)
            update_shoot_data();
        else if (command_id == 0x0208)
            update_bullet_allowance();
        else if (command_id == 0x020D)
            update_sentry_info();
        else if (command_id == 0x0303)
            update_map_command();
    }

    void update_game_status() {
        auto& data = frame_body_as<GameStatus>();

        *game_stage_ = static_cast<hcs_msgs::GameStage>(data.game_progress);
        *stage_remain_time_ = data.stage_remain_time;
        *sync_timestamp_ = data.sync_timestamp;

        if (*game_stage_ == hcs_msgs::GameStage::STARTED)
            game_status_watchdog_.reset(30'000);
        else
            game_status_watchdog_.reset(5'000);
    }

    void update_event_data() {
        auto& data = frame_body_as<EventData>();

        *ally_small_energy_activation_status_ = data.ally_small_energy_activation_status;
        *ally_big_energy_activation_status_ = data.ally_big_energy_activation_status;
        *ally_fortress_occupation_status_ = data.ally_fortress_occupation_status;
    }

    void update_dart_info() {
        auto& data = frame_body_as<DartInfo>();

        *dart_latest_hit_target_total_count_ = data.latest_hit_target_total_count;
    }

    void update_game_robot_hp() {
        auto& data = frame_body_as<GameRobotHp>();
        *robots_hp_ = data;
        *ally_hero_hp_ = data.ally_1_robot_hp;
        *ally_engineer_hp_ = data.ally_2_robot_hp;
        *ally_infantry_1_hp_ = data.ally_3_robot_hp;
        *ally_infantry_2_hp_ = data.ally_4_robot_hp;
        *ally_outpost_hp_ = data.ally_outpost_hp;
        *ally_base_hp_ = data.ally_base_hp;
        *enemy_outpost_hp_ = data.enemy_outpost_hp;
        *enemy_base_hp_ = data.enemy_base_hp;
        *damage_difference_ = data.damage_difference;
    }

    void update_robot_status() {
        if (*game_stage_ == hcs_msgs::GameStage::STARTED)
            robot_status_watchdog_.reset(60'000);
        else
            robot_status_watchdog_.reset(5'000);

        auto& data = frame_body_as<RobotStatus>();

        *robot_current_hp_ = data.current_hp;
        *robot_id_ = static_cast<hcs_msgs::RobotId>(data.robot_id);
        *robot_shooter_cooling_ = data.shooter_barrel_cooling_value;
        *robot_shooter_heat_limit_ = static_cast<int64_t>(1000) * data.shooter_barrel_heat_limit;

        if (data.chassis_power_limit == std::numeric_limits<uint16_t>::max())
            *robot_chassis_power_limit_ = std::numeric_limits<double>::infinity();
        else
            *robot_chassis_power_limit_ = static_cast<double>(data.chassis_power_limit);

        *chassis_output_status_ = data.power_management_chassis_output;
    }

    void update_power_heat_data() {
        power_heat_data_watchdog_.reset(3'000);

        auto& data = frame_body_as<PowerHeatData>();
        *robot_buffer_energy_ = static_cast<double>(data.buffer_energy);
    }

    void update_robot_position() {
        auto& data = frame_body_as<RobotPosition>();
        *robot_position_x_ = data.x;
        *robot_position_y_ = data.y;
        *robot_position_angle_ = data.angle;
    }

    void update_hurt_data() {}

    void update_shoot_data() {
        auto& data = frame_body_as<ShootData>();
        *robot_initial_speed_ = data.initial_speed;

        const auto now = std::chrono::high_resolution_clock::now();
        *robot_shoot_timestamp_ = std::chrono::duration<double>(now.time_since_epoch()).count();
    }

    void update_bullet_allowance() {
        auto& data = frame_body_as<BulletAllowance>();
        *robot_bullet_allowance_ = data.projectile_allowance_17mm;
        *robot_42mm_bullet_allowance_ = data.projectile_allowance_42mm;
        *remaining_gold_coin_ = data.remaining_gold_coin;
        *robot_fortress_17mm_bullet_allowance_ = data.projectile_allowance_fortress;
    }

    void update_sentry_info() {
        auto& data = frame_body_as<SentryInfo>();

        *sentry_posture_ = static_cast<uint8_t>(data.posture + (data.is_powered ? 3 : 0));
        *sentry_is_powered_ = data.is_powered;
        *sentry_is_disengaged_ = data.is_disengaged;
        *sentry_can_rebirth_free_ = data.can_rebirth_free;
        *sentry_can_rebirth_gold_ = data.can_rebirth_gold;
        *sentry_rebirth_gold_cost_ = data.rebirth_gold_cost;
    }

    void update_map_command() {
        if (frame_.header.data_length < sizeof(MapCommand)) {
            if (const auto count = short_map_commands_.hit())
                logger().rt().warn(
                    "Map command length invalid: {} ({} so far)",
                    static_cast<unsigned>(frame_.header.data_length), *count);
            return;
        }

        MapCommand data;
        std::memcpy(&data, frame_.body.data, sizeof(data));

        *map_command_target_position_x_ = data.target_position_x;
        *map_command_target_position_y_ = data.target_position_y;
        *map_command_keyboard_ = data.cmd_keyboard;
        *map_command_target_robot_id_ = data.target_robot_id;
        *map_command_source_ = data.cmd_source;

        const auto now = std::chrono::high_resolution_clock::now();
        *map_command_received_timestamp_ =
            std::chrono::duration<double>(now.time_since_epoch()).count();

        if (has_last_map_command_
            && std::memcmp(&last_map_command_, &data, sizeof(data)) == 0) { // NOLINT
            return;
        }

        last_map_command_ = data;
        has_last_map_command_ = true;

        *map_command_event_target_position_x_ = data.target_position_x;
        *map_command_event_target_position_y_ = data.target_position_y;
        *map_command_event_keyboard_ = data.cmd_keyboard;
        *map_command_event_target_robot_id_ = data.target_robot_id;
        *map_command_event_source_ = data.cmd_source;
        *map_command_event_timestamp_ = *map_command_received_timestamp_;
        *map_command_event_sequence_ += 1;
    }
    // When referee system loses connection unexpectedly,
    // use these indicators make sure the robot safe.
    // Muzzle: Cooling priority with level 1
    static constexpr int64_t safe_shooter_cooling = 40;
    static constexpr int64_t safe_shooter_heat_limit = 50'000;
    // Chassis: Health priority with level 1
    static constexpr double safe_chassis_power_limit = 45;

    // 线路一坏，下面这些错每拍都会来。按 1、2、4、8… 次稀释着报，而不是每拍一条。
    hcs_log::Backoff oversized_frames_;
    hcs_log::Backoff body_crc_errors_;
    hcs_log::Backoff header_start_errors_;
    hcs_log::Backoff header_crc_errors_;
    hcs_log::Backoff short_map_commands_;

    InputInterface<hcs_msgs::SerialInterface> serial_;
    Frame frame_;
    size_t cache_size_ = 0;

    hcs_utility::TickTimer game_status_watchdog_;
    OutputInterface<hcs_msgs::GameStage> game_stage_;
    OutputInterface<uint16_t> stage_remain_time_;
    OutputInterface<uint64_t> sync_timestamp_;
    OutputInterface<uint8_t> ally_big_energy_activation_status_;
    OutputInterface<uint8_t> ally_small_energy_activation_status_;
    OutputInterface<uint8_t> ally_fortress_occupation_status_;
    OutputInterface<uint8_t> dart_latest_hit_target_total_count_;

    hcs_utility::TickTimer robot_status_watchdog_;
    OutputInterface<hcs_msgs::RobotId> robot_id_;
    OutputInterface<int64_t> robot_shooter_cooling_, robot_shooter_heat_limit_;
    OutputInterface<double> robot_chassis_power_limit_;
    OutputInterface<bool> chassis_output_status_;

    OutputInterface<uint8_t> sentry_posture_;
    OutputInterface<bool> sentry_is_powered_;
    OutputInterface<bool> sentry_is_disengaged_;
    OutputInterface<bool> sentry_can_rebirth_free_;
    OutputInterface<bool> sentry_can_rebirth_gold_;
    OutputInterface<uint16_t> sentry_rebirth_gold_cost_;

    hcs_utility::TickTimer power_heat_data_watchdog_;
    OutputInterface<double> robot_chassis_power_;
    OutputInterface<double> robot_buffer_energy_;

    OutputInterface<GameRobotHp> robots_hp_;
    OutputInterface<uint16_t> ally_hero_hp_;
    OutputInterface<uint16_t> ally_engineer_hp_;
    OutputInterface<uint16_t> ally_infantry_1_hp_;
    OutputInterface<uint16_t> ally_infantry_2_hp_;
    OutputInterface<uint16_t> ally_outpost_hp_;
    OutputInterface<uint16_t> ally_base_hp_;
    OutputInterface<uint16_t> enemy_outpost_hp_;
    OutputInterface<uint16_t> enemy_base_hp_;
    OutputInterface<int16_t> damage_difference_;
    OutputInterface<uint16_t> robot_current_hp_;
    OutputInterface<double> robot_position_x_;
    OutputInterface<double> robot_position_y_;
    OutputInterface<double> robot_position_angle_;
    OutputInterface<uint16_t> robot_bullet_allowance_;
    OutputInterface<uint16_t> robot_42mm_bullet_allowance_;
    OutputInterface<uint16_t> robot_fortress_17mm_bullet_allowance_;
    OutputInterface<uint16_t> remaining_gold_coin_;

    OutputInterface<float> robot_initial_speed_;
    OutputInterface<double> robot_shoot_timestamp_;

    OutputInterface<double> map_command_target_position_x_;
    OutputInterface<double> map_command_target_position_y_;
    OutputInterface<uint8_t> map_command_keyboard_;
    OutputInterface<uint8_t> map_command_target_robot_id_;
    OutputInterface<uint16_t> map_command_source_;
    OutputInterface<double> map_command_received_timestamp_;
    OutputInterface<double> map_command_event_target_position_x_;
    OutputInterface<double> map_command_event_target_position_y_;
    OutputInterface<uint8_t> map_command_event_keyboard_;
    OutputInterface<uint8_t> map_command_event_target_robot_id_;
    OutputInterface<uint16_t> map_command_event_source_;
    OutputInterface<double> map_command_event_timestamp_;
    OutputInterface<uint64_t> map_command_event_sequence_;
    MapCommand last_map_command_{};
    bool has_last_map_command_ = false;
};

} // namespace hcs_core::referee

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hcs_core::referee::Status, hcs_executor::Component)
