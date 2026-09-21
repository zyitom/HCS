// 对拍测试主程序：同一脚本化输入喂给 Helios 参照实现（原码 + stub）与新实现
// （ChassisLogic），逐拍比较阶段序列与关键输出。
//
// 用法：见同目录 run_compare.sh；输出为逐场景统计 + 差异清单。

#include <cmath>
#include <cstdlib>
#include <memory>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../chassis_logic.hpp"

#include "helios_ref/bsp_dwt.h"
#include "helios_ref/chassis_balance_v2.hpp"

#include "scenarios.hpp"

namespace hcs_core::controller::balance::port {

struct RefOut {
    const char* mode;
    int flag;
    float ll_want[2];
    float rotate_angle[2];
    float s1;
    float leg_torque[4]; // [bl1, bl0, br1, br0]
    float wheel_current[2];
};

struct DiffStats {
    long ticks = 0;
    long mode_mismatch = 0;
    long flag_mismatch = 0;
    long ll_mismatch = 0;
    long rotate_mismatch = 0;
    long leg_torque_mismatch = 0;
    long wheel_current_mismatch = 0;
    float max_leg_torque_diff = 0.0f;
    float max_wheel_current_diff = 0.0f;
    float max_ll_diff = 0.0f;
    float max_rotate_diff = 0.0f;
    std::vector<std::string> first_diffs;
};

constexpr float kTolerance = 1e-3f;

void note_diff(DiffStats& stats, int tick, const std::string& what) {
    if (stats.first_diffs.size() < 8)
        stats.first_diffs.push_back("tick " + std::to_string(tick) + ": " + what);
}

const char* ref_mode_str(app::mode_e mode) {
    switch (mode) {
    case app::mode_e::DISABLED: return "DISABLED";
    case app::mode_e::NORMAL: return "NORMAL";
    case app::mode_e::SPIN: return "SPIN";
    case app::mode_e::SLOW_START: return "SLOW_START";
    case app::mode_e::TANK: return "TANK";
    case app::mode_e::UP_STAIR: return "UP_STAIR";
    case app::mode_e::SELF_HEAL: return "SELF_HEAL";
    case app::mode_e::JUMP: return "JUMP";
    case app::mode_e::FLY: return "FLY";
    case app::mode_e::TOUCH_DOWN: return "TOUCH_DOWN";
    }
    return "?";
}

const char* new_mode_str(const Phase& phase, bool spinning) {
    switch (kind_of(phase)) {
    case PhaseKind::Disabled: return "DISABLED";
    case PhaseKind::Fallen: return "DISABLED"; // 拆分后的 Fallen 在模式空间仍是 DISABLED
    case PhaseKind::Standing: return spinning ? "SPIN" : "NORMAL";
    case PhaseKind::SlowStart: return "SLOW_START";
    case PhaseKind::UpStair: return "UP_STAIR";
    case PhaseKind::Jump: return "JUMP";
    case PhaseKind::Fly: return "FLY";
    case PhaseKind::SelfHeal: return "SELF_HEAL";
    }
    return "?";
}

const char* flag_str(int flag) {
    switch (flag) {
    case 0: return "LQR_ON";
    case 1: return "DISABLED";
    case 2: return "PID_ONLY";
    case 3: return "HEALING";
    }
    return "?";
}

class CompareHarness {
public:
    explicit CompareHarness(bool gnd_enabled) {
        g_harness_chassis_ctrl = Chassis_Ctrl_Data_s{};
        balance_ = new app::Chassis_Balance(&bl0_, &bl1_, &br0_, &br1_, &wl_, &wr_, &imu_);
        status_ = new app::Chassis_Balance_Status_Handle(*balance_, nullptr);
        ctrl_ = new app::Chassis_Balance_Ctrl(*balance_, *status_);

        params_.gnd_detection_enabled = gnd_enabled;
        logic_ = std::make_unique<ChassisLogic>(params_);
    }

    ~CompareHarness() {
        delete ctrl_;
        delete status_;
        delete balance_;
    }

    void step(int tick, const TickInputs& in, DiffStats& stats) {
        // ── 参照侧输入 ─────────────────────────────────────────────────────
        const MotorAngles m = phi_to_motor(
            in.pose_left.phi0, in.pose_left.phi0 * 0.0f + in.pose_left.phi0, in.pose_right.phi0,
            in.pose_right.phi0);
        (void)m;
        // 由 (phi0, L0) 反解出的摆角差直接用 solve_motor_angles 的结果（带分支连续性）：
        const auto [phi0t_l, d_l] = solve_motor_angles(in.pose_left, guess_left_);
        const auto [phi0t_r, d_r] = solve_motor_angles(in.pose_right, guess_right_);
        guess_left_ = {phi0t_l, d_l};
        guess_right_ = {phi0t_r, d_r};
        const MotorAngles angles = phi_to_motor(
            phi0t_l + d_l, phi0t_l - d_l, phi0t_r + d_r, phi0t_r - d_r);

        bl1_.offset_angle = angles.bl1; bl1_.speed = angles.bl1_speed;
        bl1_.real_torque = in.leg_torque_l1;
        bl0_.offset_angle = angles.bl0; bl0_.speed = angles.bl0_speed;
        bl0_.real_torque = in.leg_torque_l0;
        br1_.offset_angle = angles.br1; br1_.speed = angles.br1_speed;
        br1_.real_torque = in.leg_torque_r1;
        br0_.offset_angle = angles.br0; br0_.speed = angles.br0_speed;
        br0_.real_torque = in.leg_torque_r0;

        wl_.speed = in.wheel_speed_l; wl_.real_current = in.wheel_current_l;
        wr_.speed = in.wheel_speed_r; wr_.real_current = in.wheel_current_r;

        imu_.roll = in.roll;
        imu_.pitch = in.pitch;
        imu_.yaw = in.yaw;
        imu_.droll = in.droll;
        imu_.dpitch = in.dpitch;
        imu_.dyaw = in.dyaw;
        imu_.total_imu_yaw = in.total_yaw;
        imu_.acc_x = in.acc_x;
        imu_.acc_y = in.acc_y;
        imu_.acc_z = in.acc_z;

        auto& g = g_harness_chassis_ctrl;
        g.if_enable = in.if_enable ? 1 : 0;
        g.speed[0] = in.speed_level[0];
        g.speed[1] = in.speed_level[1];
        g.speed[2] = in.speed_level[2];
        g.if_free = (in.speed_level[2] != 0.0f) ? 1 : 0; // fsm.cpp:148-151 的派生
        g.if_turn = in.if_turn;
        g.jump_key = in.jump_key;
        g.save_key = in.save_key;
        g.leg_key = in.leg_key;
        g.follow_angle = in.follow_angle_cmd;
        g.real_angle = in.real_angle;

        // ── 新实现输入（同一份语义） ───────────────────────────────────────
        LogicInput logic_in{};
        logic_in.imu.roll = in.roll;
        logic_in.imu.pitch = in.pitch;
        logic_in.imu.yaw = in.yaw;
        logic_in.imu.droll = in.droll;
        logic_in.imu.dpitch = in.dpitch;
        logic_in.imu.dyaw = in.dyaw;
        logic_in.imu.total_yaw = in.total_yaw;
        logic_in.imu.acc_x = in.acc_x;
        logic_in.imu.acc_y = in.acc_y;
        logic_in.imu.acc_z = in.acc_z;
        logic_in.leg_motor[0] = {angles.bl1, angles.bl1_speed, in.leg_torque_l1};
        logic_in.leg_motor[1] = {angles.bl0, angles.bl0_speed, in.leg_torque_l0};
        logic_in.leg_motor[2] = {angles.br1, angles.br1_speed, in.leg_torque_r1};
        logic_in.leg_motor[3] = {angles.br0, angles.br0_speed, in.leg_torque_r0};
        logic_in.wheel[0] = {in.wheel_speed_l, in.wheel_current_l};
        logic_in.wheel[1] = {in.wheel_speed_r, in.wheel_current_r};
        OperatorCommand& cmd = logic_in.command;
        cmd.speed_level[0] = in.speed_level[0];
        cmd.speed_level[1] = in.speed_level[1];
        cmd.speed_level[2] = in.speed_level[2];
        cmd.if_enable = in.if_enable;
        cmd.if_free = g.if_free != 0;
        cmd.if_turn = in.if_turn;
        cmd.jump_rising_edge = in.jump_key && !jump_prev_;
        cmd.save_changed = in.save_key != save_prev_;
        cmd.save_level = in.save_key;
        cmd.leg_changed = in.leg_key != leg_prev_;
        cmd.follow_angle_cmd = in.follow_angle_cmd;
        cmd.real_angle = in.real_angle;
        jump_prev_ = in.jump_key;
        save_prev_ = in.save_key;
        leg_prev_ = in.leg_key;

        // ── 各自推进一拍 ───────────────────────────────────────────────────
        dwt_advance_tick();

        balance_->Chassis_Data_Update();
        rc_dummy_ = module::RC_ctrl_t{};
        status_->FSM(&rc_dummy_);
        ctrl_->Chassis_Controller();
        ctrl_->Motor_Send();

        logic_->update(logic_in, 0.001f);

        // ── 比较 ───────────────────────────────────────────────────────────
        if (getenv("HCS_DIAG") != nullptr && tick >= std::atoi(getenv("HCS_DIAG")) && tick <= std::atoi(getenv("HCS_DIAG")) + 9) {
            std::printf(
                "[t%d] ref: LLw=%.4f rot=%.4f Fw=%.4f pidDt=%.4f Tl0=%.4f Tl1=%.4f | "
                "new: LLw=%.4f rot=%.4f Fw=%.4f Tl0=%.4f Tl1=%.4f\n",
                tick, balance_->wbc_state.LL_want[0], balance_->wbc_state.rotate_angle[0],
                ctrl_->output[0].F_want, ctrl_->pid_LL->pidinstance[0].dt,
                ctrl_->output[0].final_Tl0, ctrl_->output[0].final_Tl1,
                logic_->estimate().wbc.LL_want[0], logic_->estimate().wbc.rotate_angle[0],
                logic_->telemetry().f_want[0], logic_->telemetry().final_tl0[0],
                logic_->telemetry().final_tl1[0]);
            std::printf(
                "[r%d] ref: phi0t=%.6f rotT=%.6f | new: phi0t=%.6f rotT(line)~\n",
                tick, balance_->leg_state[0].phi0_total,
                status_->robot_state.rotate_FSM_Want[0], logic_->estimate().leg[0].phi0_total);

        }
        if (tick == 0) {
            std::printf(
                "[diag] angles bl0=%.4f bl1=%.4f | ref phi0=%.4f phi3=%.4f L0=%.4f | "
                "new phi0=%.4f phi3=%.4f L0=%.4f\n",
                angles.bl0, angles.bl1, balance_->leg_state[0].phi[0],
                balance_->leg_state[0].phi[3], balance_->leg_state[0].L0,
                logic_->estimate().leg[0].phi[0], logic_->estimate().leg[0].phi[3],
                logic_->estimate().leg[0].L0);
        }
        const auto& rs = status_->robot_state;
        const auto& ref_wbc = balance_->wbc_state;
        const auto& est = logic_->estimate();
        const auto& view = logic_->view();
        const auto& cmds = logic_->commands();

        const char* ref_mode = ref_mode_str(rs.mode[0]);
        const char* new_mode = new_mode_str(logic_->phase(), view.spinning);
        if (getenv("HCS_TRACE") != nullptr) {
            static const char* last_ref = "";
            static const char* last_new = "";
            if (std::strcmp(last_ref, ref_mode) != 0 || std::strcmp(last_new, new_mode) != 0) {
                std::printf("[trace t%d] ref=%s new=%s\n", tick, ref_mode, new_mode);
                last_ref = ref_mode;
                last_new = new_mode;
            }
        }
        ++stats.ticks;
        if (std::strcmp(ref_mode, new_mode) != 0) {
            ++stats.mode_mismatch;
            note_diff(
                stats, tick,
                std::string("mode ref=") + ref_mode + " new=" + new_mode);
        }
        if (static_cast<int>(view.process_flag) != static_cast<int>(rs.state_flag[0])) {
            ++stats.flag_mismatch;
            note_diff(
                stats, tick,
                std::string("flag ref=") + flag_str(static_cast<int>(rs.state_flag[0]))
                    + " new=" + flag_str(static_cast<int>(view.process_flag)));
        }

        const float ref_ll[2] = {ref_wbc.LL_want[0], ref_wbc.LL_want[1]};
        const float new_ll[2] = {est.wbc.LL_want[0], est.wbc.LL_want[1]};
        const float ref_rot[2] = {ref_wbc.rotate_angle[0], ref_wbc.rotate_angle[1]};
        const float new_rot[2] = {est.wbc.rotate_angle[0], est.wbc.rotate_angle[1]};
        for (int i = 0; i < 2; ++i) {
            const float dll = std::fabs(ref_ll[i] - new_ll[i]);
            if (dll > stats.max_ll_diff)
                stats.max_ll_diff = dll;
            if (dll > kTolerance) {
                ++stats.ll_mismatch;
                if (i == 0)
                    note_diff(
                        stats, tick,
                        "LL_want ref=" + std::to_string(ref_ll[i]) + " new="
                            + std::to_string(new_ll[i]));
            }
            const float drot = std::fabs(ref_rot[i] - new_rot[i]);
            if (drot > stats.max_rotate_diff)
                stats.max_rotate_diff = drot;
            if (drot > kTolerance) {
                ++stats.rotate_mismatch;
                if (i == 0)
                    note_diff(
                        stats, tick,
                        "rotate ref=" + std::to_string(ref_rot[i]) + " new="
                            + std::to_string(new_rot[i]));
            }
        }

        const float ref_leg[4] = {bl1_.send_data, bl0_.send_data, br1_.send_data, br0_.send_data};
        for (int i = 0; i < 4; ++i) {
            const float d = std::fabs(ref_leg[i] - cmds.leg_torque[static_cast<std::size_t>(i)]);
            if (d > stats.max_leg_torque_diff)
                stats.max_leg_torque_diff = d;
            if (d > kTolerance) {
                ++stats.leg_torque_mismatch;
                if (stats.leg_torque_mismatch <= 3)
                    note_diff(
                        stats, tick,
                        "leg[" + std::to_string(i) + "] ref=" + std::to_string(ref_leg[i])
                            + " new=" + std::to_string(cmds.leg_torque[static_cast<std::size_t>(i)]));
            }
        }

        const float ref_wheel[2] = {wl_.send_data, wr_.send_data};
        for (int i = 0; i < 2; ++i) {
            const float d =
                std::fabs(ref_wheel[i] - cmds.wheel_current[static_cast<std::size_t>(i)]);
            if (d > stats.max_wheel_current_diff)
                stats.max_wheel_current_diff = d;
            if (d > kTolerance) {
                ++stats.wheel_current_mismatch;
                if (stats.wheel_current_mismatch <= 3)
                    note_diff(
                        stats, tick,
                        "wheel[" + std::to_string(i) + "] ref=" + std::to_string(ref_wheel[i])
                            + " new="
                            + std::to_string(cmds.wheel_current[static_cast<std::size_t>(i)]));
            }
        }
    }

private:
    module::moto_data_receive_s bl0_{}, bl1_{}, br0_{}, br1_{}, wl_{}, wr_{};
    module::imu_data_t imu_{};
    module::RC_ctrl_t rc_dummy_{};
    app::Chassis_Balance* balance_ = nullptr;
    app::Chassis_Balance_Status_Handle* status_ = nullptr;
    app::Chassis_Balance_Ctrl* ctrl_ = nullptr;

    Params params_;
    std::unique_ptr<ChassisLogic> logic_;

    bool jump_prev_ = false;
    bool save_prev_ = false;
    bool leg_prev_ = false;
    std::pair<float, float> guess_left_{1.39f, 0.30f};
    std::pair<float, float> guess_right_{1.39f, 0.30f};
};

} // namespace hcs_core::controller::balance::port

/// 参照实现带着 Helios 的函数级 static / 文件级全局量，跨场景会互相污染
/// （这正是任务书禁用 static 的原因的活证据）——每个场景在独立进程里跑。
int run_scenario(int index) {
    using namespace hcs_core::controller::balance::port;
    const auto scenarios = make_scenarios();
    const auto& scenario = scenarios[static_cast<std::size_t>(index)];
    long total_ticks = 0, total_mode = 0, total_float = 0;

    {
        CompareHarness harness(scenario.gnd_detection_enabled);
        DiffStats stats;
        TickInputs inputs;
        for (int tick = 0; tick < scenario.ticks; ++tick) {
            inputs = TickInputs{};
            inputs.gnd_detection_enabled = scenario.gnd_detection_enabled;
            scenario.step(tick, static_cast<float>(tick) * 0.001f, inputs);
            harness.step(tick, inputs, stats);
        }

        const long float_mismatch = stats.ll_mismatch + stats.rotate_mismatch
                                  + stats.leg_torque_mismatch + stats.wheel_current_mismatch;
        total_ticks += stats.ticks;
        total_mode += stats.mode_mismatch + stats.flag_mismatch;
        total_float += float_mismatch;

        std::printf("== %s (gnd_detection=%s)\n", scenario.name,
                    scenario.gnd_detection_enabled ? "on" : "off");
        std::printf(
            "   ticks=%ld  mode_mismatch=%ld  flag_mismatch=%ld  float_mismatch=%ld\n",
            stats.ticks, stats.mode_mismatch, stats.flag_mismatch, float_mismatch);
        std::printf(
            "   max_diff: leg_torque=%.3g wheel=%.3g ll=%.3g rotate=%.3g\n",
            stats.max_leg_torque_diff, stats.max_wheel_current_diff, stats.max_ll_diff,
            stats.max_rotate_diff);
        for (const auto& line : stats.first_diffs)
            std::printf("   diff | %s\n", line.c_str());
        std::printf("\n");
        std::fflush(stdout);
    }

    return (total_mode == 0 && total_float == 0) ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: %s <scenario_index>\n", argv[0]);
        return 2;
    }
    return run_scenario(std::atoi(argv[1]));
}
