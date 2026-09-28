#!/usr/bin/env python3
"""不停机的 gdb 远程服务端：让 gdb 像 Ozone 一样看运行中的 hcs_executor。

gdb 通过 `target remote` 连上来，内存从 /proc/PID/mem 直接读。目标进程从头到尾
不被 ptrace、不被暂停，RT 线程不受影响（实测：全速读整棵对象树 63 次/秒、10 Hz
刷新监视，start_late 最大值与空闲基线相同，跳拍 0）。在 gdb 看来目标一直是"停着"
的，`continue` 会立刻返回，相当于刷新一次。

只实现了 gdb 求值需要的部分：读内存（可选写）、auxv（gdb 靠它重定位 PIE、找到各个
.so 的加载地址）、一个假线程。寄存器除 rsp/rip 外都是 0，所以调用栈看不了，看变量
不需要它。

前提：
  1. hcs_executor 与组件库用 -g 编译（两个包的 CMakeLists 已经加上）；
  2. 目标进程带 HCS_ALLOW_DEBUG_ATTACH=1 启动（Yama ptrace_scope=1 下放行读 /proc/PID/mem）。

用法：
  python3 hcs_gdbstub.py [PID] [--port 2345] [--allow-write]
  gdb  install/hcs_executor/lib/hcs_executor/hcs_executor
    (gdb) set sysroot /
    (gdb) target remote 127.0.0.1:2345
    (gdb) set language c++
    (gdb) source hcs_bringup/tools/hcs_gdb.py
    (gdb) hcs-components                 # 每个组件绑定成 $<yaml 实例名>
    (gdb) print *$demo_hardware          # 整棵对象树：基类、成员、STL、接口值
    (gdb) set var $demo_hardware->setpoint_amplitude_ = 5    # 需要 --allow-write

VS Code + Cortex-Debug（marus25.cortex-debug）launch.json：自动刷新的 Live Watch。
在 Live Watch 面板里加表达式 `*$hcs("demo_hardware")` 即可展开整棵对象树：
  {
    "name": "HCS live watch", "type": "cortex-debug", "request": "attach",
    "servertype": "external", "gdbTarget": "127.0.0.1:2345",
    "executable": "${workspaceFolder}/install/hcs_executor/lib/hcs_executor/hcs_executor",
    "gdbPath": "/usr/bin/gdb", "objdumpPath": "/usr/bin/objdump",
    "cwd": "${workspaceFolder}",
    "debuggerArgs": ["-ex", "set sysroot /", "-ex", "set language c++",
                     "-x", "${workspaceFolder}/hcs_bringup/tools/hcs_gdb.py"],
    "liveWatch": {"enabled": true, "samplesPerSecond": 10}
  }
  request 必须是 attach：launch 会先 load 程序、再 monitor reset。
  debuggerArgs 对主 gdb 和 Live Watch 那个 gdb 都生效，所以两边都能用 $hcs()。

只用 VS Code 自带的 C/C++ 扩展（ms-vscode.cpptools）也行，但只能在按 F5 时刷新：
  {
    "name": "HCS live", "type": "cppdbg", "request": "launch",
    "program": "${workspaceFolder}/install/hcs_executor/lib/hcs_executor/hcs_executor",
    "cwd": "${workspaceFolder}", "MIMode": "gdb",
    "miDebuggerServerAddress": "127.0.0.1:2345",
    "setupCommands": [
      {"text": "-enable-pretty-printing"}, {"text": "set sysroot /"},
      {"text": "set language c++"},
      {"text": "source ${workspaceFolder}/hcs_bringup/tools/hcs_gdb.py"}
    ]
  }

monitor 命令（gdb 里 `monitor ...`）：
  find-u64 HEX         在可写私有映射里找 8 字节对齐、值为 HEX 的地址（按虚表找对象用）
  halt / reset / resume  前端连上时会发，这里一律忽略（目标永远不停）

安全：写入只落在可写的数据段。gdb 在 continue 之前会往代码段写 int3 插断点（包括它
自己在动态链接器里的内部断点），这些写入全部被拒绝，所以按 F5、打断点都不会改动进程。

局限：局部变量看不到（只能看成员和输出接口）；读到的是两拍之间的值，偶尔会一半
属于上一拍；写入与 RT 线程构成数据竞争，只适合调参。
"""

import argparse
import os
import socket
import sys

# amd64 'g' packet order: rax rbx rcx rdx rsi rdi rbp rsp r8..r15 rip
_REG_COUNT = 17
_RSP, _RIP = 7, 16


def _checksum(payload: bytes) -> bytes:
    return b"%02x" % (sum(payload) & 0xFF)


def _escape_binary(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b in (0x23, 0x24, 0x7D, 0x2A):
            out += bytes((0x7D, b ^ 0x20))
        else:
            out.append(b)
    return bytes(out)


class Target:
    def __init__(self, pid: int, allow_write: bool):
        self.pid = pid
        self.allow_write = allow_write
        flags = os.O_RDWR if allow_write else os.O_RDONLY
        self.fd = os.open(f"/proc/{pid}/mem", flags)

    def read(self, addr: int, size: int) -> bytes:
        # pread takes a signed 64-bit offset; kernel-half addresses (vsyscall) cannot be read.
        if addr >= 1 << 63:
            return b""
        try:
            return os.pread(self.fd, size, addr)
        except OSError:
            return b""

    def write(self, addr: int, data: bytes) -> bool:
        # 只许写可写的数据段。/proc/PID/mem 能强行写只读的代码段，而 gdb 插软件断点
        # 就是往代码段写 int3 —— 放行的话，RT 线程下一次执行到那里就会 SIGTRAP 把进程带走。
        if not self.allow_write or addr >= 1 << 63 or not self._writable(addr, len(data)):
            return False
        try:
            return os.pwrite(self.fd, data, addr) == len(data)
        except OSError:
            return False

    def _writable(self, addr: int, size: int) -> bool:
        with open(f"/proc/{self.pid}/maps") as f:
            for line in f:
                parts = line.split()
                start, end = (int(x, 16) for x in parts[0].split("-"))
                if start <= addr and addr + size <= end:
                    return parts[1].startswith("rw")
        return False

    def auxv(self) -> bytes:
        with open(f"/proc/{self.pid}/auxv", "rb") as f:
            return f.read()

    def exe(self) -> bytes:
        return os.readlink(f"/proc/{self.pid}/exe").encode()

    def registers(self) -> bytes:
        regs = [0] * _REG_COUNT
        try:
            with open(f"/proc/{self.pid}/syscall") as f:
                fields = f.read().split()
            if len(fields) >= 9 and fields[0] != "running":
                regs[_RSP] = int(fields[-2], 16)
                regs[_RIP] = int(fields[-1], 16)
        except OSError:
            pass
        return b"".join(r.to_bytes(8, "little") for r in regs)

    def writable_private_mappings(self):
        with open(f"/proc/{self.pid}/maps") as f:
            for line in f:
                parts = line.split()
                start, end = (int(x, 16) for x in parts[0].split("-"))
                perms = parts[1]
                path = parts[5] if len(parts) > 5 else ""
                if perms.startswith("rw") and perms[3] == "p" and not path.startswith("/dev"):
                    if path in ("", "[heap]") or path.startswith("[anon"):
                        yield start, end

    def find_u64(self, value: int, limit: int = 64):
        needle = value.to_bytes(8, "little")
        chunk = 1 << 22
        found = []
        for start, end in self.writable_private_mappings():
            addr = start
            while addr < end and len(found) < limit:
                data = self.read(addr, min(chunk, end - addr))
                if not data:
                    break
                pos = data.find(needle)
                while pos != -1:
                    if (addr + pos) % 8 == 0:
                        found.append(addr + pos)
                    pos = data.find(needle, pos + 1)
                addr += len(data)
        return found


class Session:
    def __init__(self, conn: socket.socket, target: Target, verbose: bool):
        self.conn = conn
        self.target = target
        self.verbose = verbose
        self.ack = True
        self.buf = b""
        self.stop_reply = b"T05thread:%x;" % target.pid

    # ---- framing -------------------------------------------------------
    def _recv(self) -> bool:
        data = self.conn.recv(65536)
        if not data:
            return False
        self.buf += data
        return True

    def next_packet(self):
        while True:
            while self.buf[:1] in (b"+", b"-"):
                self.buf = self.buf[1:]
            if self.buf[:1] == b"\x03":
                self.buf = self.buf[1:]
                return b"\x03"
            start = self.buf.find(b"$")
            if start != -1:
                end = self.buf.find(b"#", start)
                if end != -1 and len(self.buf) >= end + 3:
                    payload = self.buf[start + 1:end]
                    self.buf = self.buf[end + 3:]
                    if self.ack:
                        self.conn.sendall(b"+")
                    return payload
            if not self._recv():
                return None

    def send(self, payload: bytes):
        if self.verbose:
            print("<-", payload[:120], file=sys.stderr)
        self.conn.sendall(b"$" + payload + b"#" + _checksum(payload))

    # ---- dispatch ------------------------------------------------------
    def serve(self):
        while True:
            pkt = self.next_packet()
            if pkt is None:
                return
            if self.verbose:
                print("->", pkt[:120], file=sys.stderr)
            if pkt == b"\x03":
                self.send(self.stop_reply)
                continue
            try:
                reply = self.handle(pkt)
            except (ValueError, OSError) as e:
                print(f"hcs_gdbstub: {pkt[:60]!r}: {e}", file=sys.stderr)
                reply = b"E01"
            if reply is None:
                return
            self.send(reply)
            if pkt == b"QStartNoAckMode":
                self.ack = False

    def handle(self, pkt: bytes):
        t = self.target
        if pkt.startswith(b"qSupported"):
            return b"PacketSize=20000;QStartNoAckMode+;qXfer:auxv:read+;qXfer:exec-file:read+"
        if pkt == b"QStartNoAckMode" or pkt == b"!":
            return b"OK"
        if pkt == b"?":
            return self.stop_reply
        if pkt == b"qC":
            return b"QC%x" % t.pid
        if pkt == b"qfThreadInfo":
            return b"m%x" % t.pid
        if pkt == b"qsThreadInfo":
            return b"l"
        if pkt == b"qAttached" or pkt.startswith(b"qAttached:"):
            return b"1"
        if pkt[:1] in (b"H", b"T"):
            return b"OK"
        if pkt == b"g":
            return t.registers().hex().encode()
        if pkt[:1] == b"m":
            addr, size = (int(x, 16) for x in pkt[1:].split(b","))
            data = t.read(addr, size)
            return data.hex().encode() if data else b"E14"
        if pkt[:1] == b"M":
            head, _, hexdata = pkt[1:].partition(b":")
            addr = int(head.split(b",")[0], 16)
            return b"OK" if t.write(addr, bytes.fromhex(hexdata.decode())) else b"E01"
        if pkt.startswith(b"qXfer:auxv:read::"):
            return self._xfer(t.auxv(), pkt[len(b"qXfer:auxv:read::"):])
        if pkt.startswith(b"qXfer:exec-file:read:"):
            rest = pkt[len(b"qXfer:exec-file:read:"):]
            return self._xfer(t.exe(), rest.split(b":", 1)[1])
        if pkt == b"vCont?":
            return b"vCont;c;C;s;S;t"
        if pkt.startswith(b"vCont") or pkt[:1] in (b"c", b"s", b"C", b"S"):
            return self.stop_reply
        if pkt.startswith(b"qSymbol"):
            return b"OK"
        if pkt.startswith(b"qRcmd,"):
            return self._monitor(bytes.fromhex(pkt[6:].decode()).decode())
        if pkt[:1] == b"D":
            self.send(b"OK")
            return None
        if pkt[:1] == b"k" or pkt.startswith(b"vKill"):
            return None
        return b""

    @staticmethod
    def _xfer(blob: bytes, span: bytes) -> bytes:
        off, length = (int(x, 16) for x in span.split(b","))
        chunk = blob[off:off + length]
        prefix = b"l" if off + length >= len(blob) else b"m"
        return prefix + _escape_binary(chunk)

    def _monitor(self, cmd: str) -> bytes:
        words = cmd.split()
        if len(words) == 2 and words[0] == "find-u64":
            hits = self.target.find_u64(int(words[1], 16))
            text = "".join("0x%x\n" % a for a in hits) or "none\n"
        elif words[:1] in (["halt"], ["reset"], ["resume"]):
            # Cortex-Debug 等前端连上时会发 monitor halt / reset halt：这里永远不停目标，照单全收。
            text = ""
        else:
            text = "commands: find-u64 HEX\n"
        if text:
            self.send(b"O" + text.encode().hex().encode())
        return b"OK"


def find_pid(exe_name: str) -> int:
    # The main thread renames itself (spin_thread_config), so match /proc/PID/exe, not comm.
    pids = []
    for entry in os.listdir("/proc"):
        if entry.isdigit():
            try:
                if os.path.basename(os.readlink(f"/proc/{entry}/exe")) == exe_name:
                    pids.append(int(entry))
            except OSError:
                continue
    if len(pids) != 1:
        sys.exit(f"hcs_gdbstub: expected one '{exe_name}' process, found {pids}; pass the pid")
    return pids[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pid", type=int, nargs="?",
                    help="target pid; default: the only process whose executable is --exe")
    ap.add_argument("--exe", default="hcs_executor",
                    help="executable basename used to find the pid (default hcs_executor)")
    ap.add_argument("--port", type=int, default=2345)
    ap.add_argument("--allow-write", action="store_true",
                    help="let gdb `set var` write into the live process")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    pid = args.pid if args.pid is not None else find_pid(args.exe)
    target = Target(pid, args.allow_write)
    with socket.create_server(("127.0.0.1", args.port)) as srv:
        print(f"hcs_gdbstub: pid {pid} on 127.0.0.1:{args.port}", file=sys.stderr)
        while True:
            conn, _ = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with conn:
                Session(conn, target, args.verbose).serve()


if __name__ == "__main__":
    main()
