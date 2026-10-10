#pragma once

// clang 的 function effect analysis：override 一个 nonblocking 虚函数的实现必须也是 nonblocking。
// 把 HCS_NONBLOCKING 标在基类那个纯虚 update() 上，编译器就替我们把"周期域里不许分配、不许
// 加锁、不许进内核"这条规矩强制到所有组件身上——包括以后新来的人写的那些，包括他们调用的模板
// 深处。虚函数在这里不是成本，是抓手：一次虚调用换来整棵实现树的静态约束。
//
// 探测必须走 __has_cpp_attribute 而不是判 __clang__ 的版本号：gcc 14.2 见到未知属性会报
// -Wattributes，而本工作区要求 -Wall -Wextra -Wpedantic 下零警告。
//
// 两条使用须知：
//   1. 属性作用于函数**类型**，所以写在形参表之后、= 0 / override 之前：
//        virtual void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING = 0;
//   2. gcc 上这两个宏展开为空，约束退化成文档。真正的检查只在 clang 那一趟成立，而且
//      -Wfunction-effects **不在** -Wall/-Wextra 里，必须显式加；只加 -Wall 的话，
//      clang 只会给出 -Wperf-constraint-implies-noexcept 这类风格提醒，不做效果推断。

#if defined(__has_cpp_attribute)
#  if __has_cpp_attribute(clang::nonblocking)
#    define HCS_NONBLOCKING [[clang::nonblocking]]
#  endif
#  if __has_cpp_attribute(clang::nonallocating)
#    define HCS_NONALLOCATING [[clang::nonallocating]]
#  endif
#endif

// 不支持的编译器上退化为空，保证同一份源码到处都能编过。
#ifndef HCS_NONBLOCKING
#  define HCS_NONBLOCKING
#endif

#ifndef HCS_NONALLOCATING
#  define HCS_NONALLOCATING
#endif
