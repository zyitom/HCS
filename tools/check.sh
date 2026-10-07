#!/usr/bin/env bash
# 一条命令回答"这份代码现在有没有问题"。提交前跑；改了 libhcs 或跨线程的代码时带上对应的开关。
#
#   tools/check.sh              构建全部包 + 跑全部测试（gcc）。日常就是这一条。
#   tools/check.sh --asan       另用 AddressSanitizer + UBSan 跑硬件层的测试（越界、释放后使用、未定义行为）
#   tools/check.sh --tsan       另用 ThreadSanitizer 跑硬件层的测试（数据竞争）
#   tools/check.sh --clang      另用 clang 构建并跑测试：-Wfunction-effects 检查周期域里有没有会阻塞的调用
#   tools/check.sh --firmware   另编 libhcs 的全部固件（改了 libhcs 的共享头或固件时必跑：主机和固件共用一份协议）
#   tools/check.sh --all        以上全部
#
# 测试分三种，都在这一条命令里：
#   纯逻辑      假板卡、假 SDK，不要硬件（hcs_core/test/test_board.cpp 等）
#   协议        libhcs 的线格式与 EP0 握手，对着模拟板（hcs_core/src/libhcs/host/tests/）
#   上板        两块 HPM5321 对接、或一块 HPM5321 对一块 mc02 的台架（hcs_core/test/test_bench_boards.cpp）；
#               板子凑不成台架时自动跳过，结果里记为 skipped。插着板子跑不过 = 真有问题
#               （先看是不是固件没重刷、线有没有接对）。
#
# 构建产物全部放在仓库外：默认 ~/.cache/hcs-check，可用 HCS_CHECK_DIR 改。
# 退出码非零 = 有一步没过，最后一行说是哪一步。

set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OUT=${HCS_CHECK_DIR:-$HOME/.cache/hcs-check}
JOBS=${HCS_CHECK_JOBS:-$(nproc)}

# 用检查器跑的测试：硬件层（跨线程交接、无锁通道、板组件生命周期）和 libhcs 的协议测试。
SANITIZED_TESTS=(test_board test_board_transmitter test_safety_latch
                 test_libhcs_wire_protocol test_libhcs_ep0_declaration
                 test_libhcs_sof_stamp test_libhcs_time_affine test_libhcs_sample_time
                 test_libhcs_time_axis test_libhcs_board_ownership test_libhcs_session)

want_asan=0 want_tsan=0 want_clang=0 want_firmware=0
for argument in "$@"; do
    case $argument in
    --asan) want_asan=1 ;;
    --tsan) want_tsan=1 ;;
    --clang) want_clang=1 ;;
    --firmware) want_firmware=1 ;;
    --all) want_asan=1 want_tsan=1 want_clang=1 want_firmware=1 ;;
    -h | --help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown option: $argument (see --help)" >&2; exit 2 ;;
    esac
done

mkdir -p "$OUT"
current_step=""
step() {
    current_step=$1
    printf '\n\033[1m== %s\033[0m\n' "$current_step"
}
trap 'status=$?; if [ $status -ne 0 ]; then printf "\n\033[31mFAILED: %s\033[0m (logs under %s)\n" "$current_step" "$OUT" >&2; fi' EXIT

# ROS 的 setup 脚本会读未定义的变量。
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u

# colcon 构建 + 测试。$1 = 产物子目录，其余 = 传给 cmake 的参数。
colcon_build_and_test() {
    local name=$1
    shift
    local bases=(--build-base "$OUT/$name/build" --install-base "$OUT/$name/install")
    (cd "$OUT" && colcon --log-base "$OUT/$name/log" build --base-paths "$ROOT" "${bases[@]}" \
        --event-handlers console_direct- --cmake-args -DBUILD_TESTING=ON "$@") \
        > "$OUT/$name-build.log" 2>&1 || { tail -40 "$OUT/$name-build.log"; return 1; }
    (cd "$OUT" && colcon --log-base "$OUT/$name/log" test --base-paths "$ROOT" "${bases[@]}" \
        --event-handlers console_direct-) > "$OUT/$name-test.log" 2>&1 || true
    # test-result 在有失败时返回非零，并列出失败的用例。
    colcon --log-base "$OUT/$name/log" test-result --test-result-base "$OUT/$name/build" --verbose \
        > "$OUT/$name-result.log" 2>&1 || { cat "$OUT/$name-result.log"; return 1; }
    tail -1 "$OUT/$name-result.log"
}

# 用检查器编 hcs_core 的那几个测试并逐个运行。$1 = 名字，$2 = 编译/链接选项，其余 = 运行前缀。
sanitized() {
    local name=$1 flags=$2
    shift 2
    set +u
    # shellcheck disable=SC1091
    source "$OUT/gcc/install/setup.bash" # 依赖包（hcs_executor 等）用普通构建的那一份
    set -u
    cmake -S "$ROOT/hcs_core" -B "$OUT/$name" -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_CXX_FLAGS="$flags" -DCMAKE_C_FLAGS="$flags" -DCMAKE_EXE_LINKER_FLAGS="$flags" \
        > "$OUT/$name-configure.log" 2>&1 || { tail -30 "$OUT/$name-configure.log"; return 1; }
    cmake --build "$OUT/$name" --target "${SANITIZED_TESTS[@]}" -j "$JOBS" \
        > "$OUT/$name-build.log" 2>&1 || { tail -40 "$OUT/$name-build.log"; return 1; }
    local test
    for test in "${SANITIZED_TESTS[@]}"; do
        if (cd "$OUT/$name" && "$@" "./$test" > "$OUT/$name-$test.log" 2>&1); then
            printf '  %-34s %s\n' "$test" "$(grep -E '^\[  PASSED  \]' "$OUT/$name-$test.log" | tail -1)"
        else
            grep -E -A25 'ERROR: |WARNING: ThreadSanitizer|runtime error|FAILED' "$OUT/$name-$test.log" | head -60
            echo "  $test: see $OUT/$name-$test.log"
            return 1
        fi
    done
}

step "build + all tests (gcc)"
colcon_build_and_test gcc

if [ $want_asan -eq 1 ]; then
    step "AddressSanitizer + UBSan"
    ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
        sanitized asan "-fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer" env
fi

if [ $want_tsan -eq 1 ]; then
    step "ThreadSanitizer"
    # 这台机器的内核开着高熵 ASLR，TSan 的影子内存映射会撞上（"unexpected memory mapping"）：关掉 ASLR 跑。
    TSAN_OPTIONS=halt_on_error=1 sanitized tsan "-fsanitize=thread" setarch -R
fi

if [ $want_clang -eq 1 ]; then
    step "clang build + tests (-Wfunction-effects)"
    clangxx=$(command -v clang++-20 || command -v clang++)
    clangc=$(command -v clang-20 || command -v clang)
    colcon_build_and_test clang -DCMAKE_CXX_COMPILER="$clangxx" -DCMAKE_C_COMPILER="$clangc"
    # 周期域里会阻塞的调用。硬件层和测试必须是 0 条；referee/ 与 hcs_link_probe.cpp 里的是已知的存量。
    offenders=$(grep -h 'Wfunction-effects' "$OUT"/clang/log/latest_build/*/stdout_stderr.log 2> /dev/null \
        | grep -v -E 'src/referee/|hcs_link_probe\.cpp' || true)
    if [ -n "$offenders" ]; then
        echo "$offenders" | head -20
        echo "blocking calls on the periodic path (above)"
        exit 1
    fi
fi

if [ $want_firmware -eq 1 ]; then
    step "libhcs firmware (mc02, hpm5321, hpm6e8y, c_board)"
    LIBHCS=$ROOT/hcs_core/src/libhcs
    export GNURISCV_TOOLCHAIN_PATH=${GNURISCV_TOOLCHAIN_PATH:-$HOME/3rd_party/rv32imac_zicsr_zifencei_multilib_b_ext-linux}
    firmware() { # $1 = 产物名，$2 = 固件目录，$3 = target，其余 = cmake 参数
        local name=$1 source=$2 target=$3
        shift 3
        cmake --preset release -S "$LIBHCS/firmware/$source" -B "$OUT/firmware-$name" "$@" \
            > "$OUT/firmware-$name.log" 2>&1 \
            && cmake --build "$OUT/firmware-$name" --target "$target" >> "$OUT/firmware-$name.log" 2>&1 \
            || { tail -30 "$OUT/firmware-$name.log"; return 1; }
        printf '  %-10s ok\n' "$name"
    }
    firmware mc02 mc02 mc02_app
    firmware hpm5321 hpm_board hpm_board_app -DBOARD=hpm5321
    firmware hpm6e8y hpm_board hpm_board_app -DBOARD=hpm6e8y
    firmware c_board c_board c_board_app
fi

step "done"
printf '\033[32mall checks passed\033[0m\n'
