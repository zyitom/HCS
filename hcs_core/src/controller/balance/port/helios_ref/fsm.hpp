// 对拍 harness 专用 stub：module::FSM（v2 仅 include；云台板的状态机不参与对拍）。
#pragma once

namespace module {

class FSM {
public:
    FSM(void*, void*) {}
    void FSM_handle() {}
};

} // namespace module
