#pragma once

// 与 hcs_utility/rt_attributes.hpp 的 HCS_NONBLOCKING 同义。本包只依赖标准库和 POSIX，
// 视觉进程也要用，所以这里单独定义一份，不去引 hcs_utility。
//
// 属性作用于函数类型：写在 noexcept 之后。gcc 上展开为空，检查只在 clang 加
// -Wfunction-effects 的那一趟成立。

#if defined(__has_cpp_attribute)
#  if __has_cpp_attribute(clang::nonblocking)
#    define HCS_LINK_NONBLOCKING [[clang::nonblocking]]
#  endif
#endif

#ifndef HCS_LINK_NONBLOCKING
#  define HCS_LINK_NONBLOCKING
#endif
