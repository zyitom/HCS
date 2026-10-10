// 独立的机器护栏检查工具：打印 hcs_utility::MachineGuard 的全部发现，
// 有 kCritical 时退出码为 1（可当 launch 前置检查或 CI 探针用）。
//
//   ros2 run hcs_bringup hcs_machine_check
//
// 全部只读，不需要 root。需要 root 的修改项见同目录 hcs_rt_tune.sh。

#include <cstdio>

#include <hcs_base/check/machine_guard.hpp>

int main() {
    const auto findings = hcs_utility::MachineGuard::run();
    for (const auto& finding : findings) {
        const char* tag = "ok  ";
        switch (finding.level) {
        case hcs_utility::MachineGuard::Level::kCritical: tag = "CRIT"; break;
        case hcs_utility::MachineGuard::Level::kWarn: tag = "WARN"; break;
        case hcs_utility::MachineGuard::Level::kInfo: tag = "info"; break;
        case hcs_utility::MachineGuard::Level::kOk: break;
        }
        std::printf("[%s] %s\n", tag, finding.text.c_str());
    }
    return hcs_utility::MachineGuard::worst_level(findings) >=
                   static_cast<int>(hcs_utility::MachineGuard::Level::kCritical)
               ? 1
               : 0;
}
