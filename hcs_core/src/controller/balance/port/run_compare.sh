#!/usr/bin/env bash
# 对拍测试：Helios 原码参照 vs ChassisLogic 新实现。
set -e
cd "$(dirname "$0")"
# 参照代码是 Helios 原码，固有警告不修（-w 只关它自己）；
# 新实现保持 -Wall -Wextra -Wpedantic 全开。
g++ -std=c++23 -O2 -g -w \
    -I . -I .. -I helios_ref \
    helios_ref/chassis_balance_v2.cpp helios_ref/balance_algorithm.cpp \
    helios_ref/pid.cpp helios_ref/user_lib.cpp helios_ref/balance_nlmpc.cpp \
    -c
g++ -std=c++23 -O2 -g -Wall -Wextra -Wpedantic \
    -I . -I .. -I helios_ref \
    compare_main.cpp \
    ../leg_model.cpp ../speed_kf.cpp ../nlmpc.cpp ../estimator.cpp \
    ../phase_machine.cpp ../controller.cpp ../operator_input.cpp \
    chassis_balance_v2.o balance_algorithm.o pid.o user_lib.o balance_nlmpc.o \
    -o /tmp/hcs_compare
# 汇总所有场景（每个场景独立进程，隔离 Helios 参照码的 static 污染）
total=0
for i in $(seq 0 11); do
    /tmp/hcs_compare "$i" || total=$((total+1))
done
exit $total
