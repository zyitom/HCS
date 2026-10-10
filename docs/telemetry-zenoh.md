# 用 zenoh 在笔记本上看机器人的话题

> **文档类型**：现行说明
> **适用范围**：`hcs_bringup/launch/hcs.launch.py` 的 `rmw:=zenoh`，ROS 2 Jazzy
> **状态**：本机已验证（2026-10-07）；笔记本那一侧还没在真机器上试过
> **相关文档**：[logging.md](logging.md) · `hcs_core/src/broadcaster/value_broadcaster.cpp`

## 摘要

DDS 在 Wi-Fi 上靠组播发现，经常发现不到或者把无线带宽打满。zenoh 改用"每台机器一个路由、路由之间连
TCP"，笔记本只要知道机器人的 IP。HCS 的控制线程不碰中间件（话题由 `ValueBroadcaster` 的非实时线程发），
所以换中间件不影响控制回路。默认（`rmw:=default`）行为不变。

## 1. 两台机器都要装

```bash
sudo apt install ros-jazzy-rmw-zenoh-cpp
```

两边的 `RMW_IMPLEMENTATION` 必须一致：rmw_zenoh_cpp 和 DDS 互相看不见。

## 2. 机器人

```bash
ros2 launch hcs_bringup hcs.launch.py robot:=balance-infantry rmw:=zenoh
```

launch 会起一个 zenoh 路由（`rmw_zenohd`，监听 `tcp/[::]:7447`，即所有网卡），executor 设成
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`，连本机路由。防火墙要放行 TCP 7447。

## 3. 笔记本

本机也起一个路由，让它连机器人的路由；之后本机的 `ros2` 命令照常用：

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<机器人 IP>:7447"]' ros2 run rmw_zenoh_cpp rmw_zenohd
```

另开一个终端（同样 `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`）：

```bash
ros2 topic list
ros2 topic echo /rosout
ros2 param set /<组件名> <参数> <值>
```

## 4. 验证记录

- 本机（AD600，2026-10-07）：`rmw:=zenoh` 起路由与 executor，另一进程以 rmw_zenoh_cpp 列出话题并
  `ros2 topic echo --once /rosout` 收到 `hcs_executor` 的日志行。[实测]（那次 executor 本身随后因安装产物
  过旧退出，与中间件无关。）
- 笔记本经 Wi-Fi：未测。
