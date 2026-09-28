"""hcs_gdbstub 的 gdb 端辅助：`target remote` 之后 `source hcs_gdb.py`。

  hcs-components   找出所有存活的 hcs_executor::Component，按 yaml 实例名绑定成
                   gdb 变量：`print *$demo_hardware`，或在 VS Code Watch 里写 `*$demo_hardware`
  hcs-find TYPE    列出动态类型为 TYPE 的存活对象
  $hcs("name")     gdb 函数：按 yaml 实例名返回组件指针，第一次调用时才扫描并缓存。
                   任何 gdb 会话（包括 Cortex-Debug 的 Live Watch 那个）只要加载了本脚本
                   就能用：Live Watch 里写 `*$hcs("demo_hardware")`

同时注册 pretty printer：OutputInterface<T> / InputInterface<T> / DelayedInput<T>
直接显示值，而不是底层存储字节。

对象是按虚表指针扫可写内存找到的，程序里不需要登记任何东西；代价是
hcs-components 连上时要扫一遍堆（demo 上约 5 秒）。
"""

import re

import gdb
import gdb.printing

_COMPONENT = "hcs_executor::Component"


def _derives_from(t, base_name, depth=0):
    t = t.strip_typedefs()
    if t.name == base_name:
        return True
    if depth > 16 or t.code != gdb.TYPE_CODE_STRUCT:
        return False
    return any(f.is_base_class and _derives_from(f.type, base_name, depth + 1) for f in t.fields())


def find_objects(type_name):
    vtable = int(gdb.parse_and_eval(f"(long)&'vtable for {type_name}'"))
    out = gdb.execute(f"monitor find-u64 {vtable + 16:x}", to_string=True)
    return [int(x, 16) for x in re.findall(r"0x[0-9a-f]+", out)]


def _component_types():
    out = gdb.execute("info variables -q ^vtable for ", to_string=True)
    for type_name in sorted(set(re.findall(r"vtable for ([\w:<>, ]+?)\s*;?$", out, re.M))):
        try:
            if _derives_from(gdb.lookup_type(type_name), _COMPONENT):
                yield type_name
        except gdb.error:
            continue


# 实例名 -> (类型名, 对象地址)。$hcs() 用它，避免每次求值都扫堆。
_components = {}


def bind_components(verbose=True):
    found = []
    _components.clear()
    for type_name in _component_types():
        for addr in find_objects(type_name):
            ptr = gdb.parse_and_eval(f"({type_name}*)0x{addr:x}")
            name = ptr.cast(gdb.lookup_type(_COMPONENT).pointer()).dereference()["component_name_"]
            name = name.format_string().strip('"')
            _components[name] = (type_name, addr)
            var = re.sub(r"\W", "_", name)
            gdb.set_convenience_variable(var, ptr)
            found.append((var, type_name, addr))
    if verbose:
        for var, type_name, addr in found:
            print(f"${var:30s} {type_name} @ 0x{addr:x}")
    return found


def _still_valid(type_name, addr):
    # 目标重启过的话地址会失效：对象首 8 字节必须仍是该类型的虚表指针。
    try:
        vptr = int(gdb.parse_and_eval(f"*(long*)0x{addr:x}"))
        return vptr == int(gdb.parse_and_eval(f"(long)&'vtable for {type_name}'")) + 16
    except gdb.error:
        return False


class HcsFunction(gdb.Function):
    """$hcs("instance_name"): pointer to the live component with that yaml name."""

    def __init__(self):
        super().__init__("hcs")

    def invoke(self, name):
        key = name.string()
        entry = _components.get(key)
        if entry is None or not _still_valid(*entry):
            bind_components(verbose=False)
            entry = _components.get(key)
        if entry is None:
            raise gdb.GdbError(f"no live component named '{key}'")
        type_name, addr = entry
        return gdb.parse_and_eval(f"({type_name}*)0x{addr:x}")


class HcsComponents(gdb.Command):
    """hcs-components: bind every live component to $<instance_name>."""

    def __init__(self):
        super().__init__("hcs-components", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        bind_components()


class HcsFind(gdb.Command):
    """hcs-find TYPE: list live objects whose dynamic type is TYPE."""

    def __init__(self):
        super().__init__("hcs-find", gdb.COMMAND_DATA)

    def invoke(self, arg, from_tty):
        type_name = arg.strip()
        for addr in find_objects(type_name):
            print(f"(({type_name}*)0x{addr:x})")


class _OutputPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        if not bool(self.val["activated"]):
            return "<inactive>"
        t = self.val.type.strip_typedefs().template_argument(0)
        return self.val["data_"].address.cast(t.pointer()).dereference()


class _InputPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        p = self.val["data_pointer_"]
        return "<unbound>" if int(p) == 0 else p.dereference()


class _DelayedPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return self.val["value_"]


def _build_printers():
    pp = gdb.printing.RegexpCollectionPrettyPrinter("hcs")
    pp.add_printer("OutputInterface", r"^hcs_executor::Component::OutputInterface<.*>$", _OutputPrinter)
    pp.add_printer("InputInterface", r"^hcs_executor::Component::InputInterface<.*>$", _InputPrinter)
    pp.add_printer("DelayedInput", r"^hcs_executor::Component::DelayedInput<.*>$", _DelayedPrinter)
    return pp


gdb.printing.register_pretty_printer(None, _build_printers(), replace=True)
HcsComponents()
HcsFind()
HcsFunction()
