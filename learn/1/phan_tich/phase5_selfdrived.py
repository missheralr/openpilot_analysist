"""
phase5_selfdrived.py — Sinh ra moi con so cua Phase 5 trong tai lieu
(may trang thai bat/tat, he thong su kien, lop an toan panda).

    python phase5_selfdrived.py <rlog.zst> <repo_fork> [repo_goc_comma]

  5.1  may trang thai va SOFT_DISABLE_TIME
  5.2  he thong su kien — su kien nao mang co gi, hau qua ra sao
  5.3  panda kiem tra lai bang C
  5.4  lop an toan cua VinFast trong kho ma cong khai chi la vo rong
  8.1  bay schema: doc log fork bang schema goc thi ten enum sai
"""
import os
import sys
import numpy as np
from collections import Counter


def show(repo, rel, pattern, n=3):
    f = os.path.join(repo, rel)
    if not os.path.isfile(f):
        f = os.path.join(repo, "openpilot", rel)
    if not os.path.isfile(f):
        print(f"    (khong thay {rel})"); return
    for i, line in enumerate(open(f, encoding="utf-8", errors="replace"), 1):
        if pattern in line:
            print(f"    {rel}:{i}  {line.rstrip()}")
            n -= 1
            if n <= 0:
                return


def main(rlog, repo, upstream=None):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import oplog
    # Uu tien bien moi truong — de dung duoc ca khi schema nam rieng mot cho,
    # khong nhat thiet phai co ca repo ma nguon.
    oplog.CEREAL = os.environ.get("CEREAL_DIR") or os.path.join(repo, "cereal")
    oplog.OPENDBC = os.environ.get("OPENDBC_DIR") or os.path.join(repo, "opendbc_repo/opendbc/car")
    lg = oplog.Log(rlog)

    print("\n" + "=" * 78)
    print("5.1  MAY TRANG THAI")
    print("=" * 78)
    show(repo, "selfdrive/selfdrived/state.py", "SOFT_DISABLE_TIME", n=1)
    show(repo, "selfdrive/selfdrived/state.py", "ACTIVE_STATES", n=1)
    ss = lg.get("selfdriveState")
    print(f"\n    Trang thai do tren log: {dict(Counter(str(e.selfdriveState.state) for e in ss))}")

    print("\n" + "=" * 78)
    print("5.2  HE THONG SU KIEN")
    print("=" * 78)
    ev = Counter()
    flags = {}
    for e in lg.get("onroadEvents"):
        for x in e.onroadEvents:
            ev[str(x.name)] += 1
            if str(x.name) not in flags:
                flags[str(x.name)] = [k for k, v in x.to_dict().items() if v is True]
    print(f"    {'su kien':22s} {'so lan':>7}  co mang theo")
    for k, n in ev.most_common():
        print(f"    {k:22s} {n:>7}  {flags[k] if flags[k] else 'khong co co nao'}")
    print("\n    cruiseMismatch khong mang co nao vi dinh nghia cua no bi chu thich tat:")
    show(repo, "selfdrive/selfdrived/events.py", "#ET.PERMANENT", n=1)

    print("\n" + "=" * 78)
    print("5.3  LOP AN TOAN PANDA")
    print("=" * 78)
    ps = lg.get("pandaStates")
    if ps:
        p0 = ps[0].pandaStates[0]
        print(f"    safetyModel = {p0.safetyModel}, safetyParam = {p0.safetyParam}, loai panda = {p0.pandaType}")
        print(f"    controlsAllowed bat {100*np.mean([e.pandaStates[0].controlsAllowed for e in ps]):.0f}% thoi gian")
    show(upstream or repo, "opendbc/safety/lateral.h", "bool steer_angle_cmd_checks", n=1)
    show(upstream or repo, "opendbc/safety/lateral.h", "rt_angle_rate_limit_check", n=1)

    print("\n" + "=" * 78)
    print("5.4  AN TOAN VINFAST TRONG KHO MA CONG KHAI")
    print("=" * 78)
    stub = os.path.join(repo, "opendbc_repo/opendbc/safety/modes/vinfast_stub.h")
    if os.path.isfile(stub):
        for line in open(stub):
            if any(s in line for s in ("controls_allowed = true", "return true", "enforcement lives")):
                print("   ", line.rstrip())
    print("    -> vo rong cho moi ban tin di qua; kiem tra that nam trong firmware da nap")

    print("\n" + "=" * 78)
    print("8.1  BAY SCHEMA — vi sao phai doc log bang schema cua chinh ban fork")
    print("=" * 78)
    def enum_at(capnp_path, idx, enum_name="SafetyModel"):
        """Tim muc thu idx BEN TRONG dung khoi enum can xem, khong phai enum dau tien gap."""
        try:
            lines = open(capnp_path, encoding="utf-8", errors="replace").read().splitlines()
        except OSError:
            return None
        inside = False
        for line in lines:
            t = line.strip()
            if t.startswith(f"enum {enum_name}"):
                inside = True
                continue
            if inside:
                if t.startswith("}"):
                    break
                if t.endswith(f"@{idx};"):
                    return t.split()[0]
        return None
    for lbl, base in (("ban fork", repo), ("ban goc", upstream)):
        if base is None:
            continue
        cap = os.path.join(base, "opendbc_repo/opendbc/car/car.capnp")
        if not os.path.isfile(cap):
            cap = os.path.join(base, "openpilot/opendbc_repo/opendbc/car/car.capnp")
        print(f"    {lbl}: chi so 35 -> {enum_at(cap,35)}   | chi so 36 -> {enum_at(cap,36)}")
    cp = lg.first("carParams")
    print(f"\n    Doc bang schema fork: safetyModel = {cp.safetyConfigs[0].safetyModel},"
          f" carFingerprint = {cp.carFingerprint}")
    print("\n    Cac ban tin de bi 'bien mat' neu dung nham schema:")
    for n in ("liveDelay", "liveParameters", "liveTracks", "liveTorqueParameters", "livePose"):
        print(f"      {n:22s} {len(lg.get(n)):>6} ban tin")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    up = os.path.expanduser(sys.argv[3]) if len(sys.argv) > 3 else None
    main(os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]), up)
