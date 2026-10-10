#pragma once

#include <cstddef>
#include <functional>

namespace hcs_msgs {

struct SerialInterface {
    std::function<size_t(std::byte*, size_t)> read;
    std::function<size_t(const std::byte*, size_t)> write;
};

} // namespace hcs_msgs