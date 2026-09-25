"""
decompile_consts.py — Lay hang so ra tu cac module bi dong goi thanh bytecode.

Ban fork dong goi mot so module thanh .pyc va xoa ma nguon: radard, latcontrol_angle,
radar_path_nudge. Nhung hang so cua Phase 2 (bo PID cua VF6), Phase 4 (hon 150 hang so
cho giao thong do thi) va bang ban va o muc 6 cua tai lieu deu lay tu day.

    python3.12 decompile_consts.py <repo_fork> [ten_module]

BAT BUOC chay bang python3.12 — dung phien ban khac se bao IndexError vi dinh dang
bytecode moi phien ban Python mot khac. Kiem tra bang: python3.12 --version

Vi du:
    python3.12 decompile_consts.py ~/qmpilot
    python3.12 decompile_consts.py ~/qmpilot latcontrol_angle_impl
"""
import dis
import marshal
import os
import sys

MODULES = {
    "radard_impl": "selfdrive/controls/__pycache__",
    "latcontrol_angle_impl": "selfdrive/controls/lib/__pycache__",
    "radar_path_nudge_impl": "selfdrive/controls/lib/__pycache__",
}


def module_constants(path):
    """Doc hang so muc module: bat cap lenh LOAD_CONST + STORE_NAME."""
    code = marshal.loads(open(path, "rb").read()[16:])   # bo 16 byte header cua .pyc
    prev, out = None, {}
    for ins in dis.get_instructions(code):
        if ins.opname == "LOAD_CONST":
            prev = ins.argval
        elif ins.opname == "STORE_NAME" and isinstance(prev, (int, float, str, bool, tuple, list, frozenset)):
            out[ins.argval] = prev
    return out, code


def functions(code):
    """Liet ke ten ham co trong module — de xac nhan no co dung nhung ham ta tuong."""
    def walk(c):
        for k in c.co_consts:
            if hasattr(k, "co_name"):
                yield k.co_name
                yield from walk(k)
    return list(walk(code))


def main(repo, only=None):
    if sys.version_info[:2] != (3, 12):
        print(f"CANH BAO: dang chay Python {sys.version_info.major}.{sys.version_info.minor}, "
              f"cac tep .pyc nay build cho 3.12 — rat co the se loi.\n")
    for name, sub in MODULES.items():
        if only and only != name:
            continue
        p = os.path.join(repo, sub, f"{name}.cpython-312.pyc")
        print("=" * 78)
        print(f"{name}   ({p})")
        print("=" * 78)
        if not os.path.isfile(p):
            print("  khong tim thay tep\n"); continue
        try:
            consts, code = module_constants(p)
        except Exception as e:
            print(f"  khong doc duoc: {type(e).__name__}: {e}\n"); continue
        ups = {k: v for k, v in consts.items() if k.isupper()}
        print(f"  {len(ups)} hang so muc module:")
        for k, v in ups.items():
            print(f"    {k:42s} = {v}")
        fns = functions(code)
        print(f"\n  {len(fns)} ham/khoi ma trong module:")
        print("    " + ", ".join(fns[:40]) + (" ..." if len(fns) > 40 else ""))
        print()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    main(os.path.expanduser(sys.argv[1]), sys.argv[2] if len(sys.argv) > 2 else None)
