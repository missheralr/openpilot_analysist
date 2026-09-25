"""
tim_schema.py — Tim xem may ban da co schema capnp nao doc duoc rlog chua.

    python3 tim_schema.py <duong_dan_rlog.zst> [thu_muc_can_quet]

Doc rlog bat buoc phai co hai tep mo ta dinh dang: cereal/log.capnp va car.capnp.
Khong nhat thiet phai clone ca repo — chi can hai tep do cung vai tep chung.
Script nay quet may ban, thu tung bo schema tim duoc, va bao cho biet bo nao
dung duoc, bo nao la cua ban fork, bo nao la cua ban goc comma.

Chay xong no in san dong lenh export de ban dan thang vao terminal.
"""

import os
import sys
import glob


def is_fork(car_capnp):
    """Ban fork co them vinfast trong danh sach SafetyModel; ban goc thi khong."""
    try:
        return "vinfast" in open(car_capnp, encoding="utf-8", errors="replace").read()
    except OSError:
        return False


def candidates(home):
    """Moi bo gom (thu_muc_cereal, thu_muc_chua_car.capnp)."""
    out = []
    for logc in glob.glob(os.path.join(home, "**", "cereal", "log.capnp"), recursive=True):
        cer = os.path.dirname(logc)
        # car.capnp co the nam ngay trong cereal (thuong la symlink) hoac trong opendbc
        here = os.path.join(cer, "car.capnp")
        if os.path.exists(here):
            real = os.path.realpath(here)
            out.append((cer, os.path.dirname(real)))
            continue
        root = os.path.dirname(cer)
        for sub in ("opendbc_repo/opendbc/car", "opendbc/car",
                    "../opendbc_repo/opendbc/car", "../opendbc/opendbc/car"):
            p = os.path.normpath(os.path.join(root, sub))
            if os.path.isfile(os.path.join(p, "car.capnp")):
                out.append((cer, p))
                break
    # bo trung
    seen, uniq = set(), []
    for c in out:
        if c not in seen:
            seen.add(c); uniq.append(c)
    return uniq


def try_schema(cer, dbc, rlog):
    """Thu doc that su rlog bang bo schema nay, trong mot tien trinh rieng."""
    import subprocess
    code = (
        "import sys, os\n"
        f"sys.path.insert(0, {os.path.dirname(os.path.abspath(__file__))!r})\n"
        "import oplog\n"
        f"oplog.CEREAL = {cer!r}\n"
        f"oplog.OPENDBC = {dbc!r}\n"
        f"lg = oplog.Log({rlog!r})\n"
        "cp = lg.first('carParams')\n"
        "print(len(lg.events), cp.carFingerprint, cp.safetyConfigs[0].safetyModel, "
        "len(lg.get('liveDelay')), len(lg.get('liveTracks')))\n"
    )
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        return None, (r.stderr.strip().splitlines() or ["loi khong ro"])[-1][:90]
    return r.stdout.strip().split(), None


def main(rlog, home=None):
    home = home or os.path.expanduser("~")
    print(f"Dang quet {home} de tim log.capnp ...\n")
    cands = candidates(home)
    if not cands:
        print("Khong tim thay bo schema nao tren may.")
        print("\nCach lay ma khong can clone ca repo — chi 5 tep, khoang 124 KB:")
        print(show_download())
        return 1

    good = []
    for cer, dbc in cands:
        tag = "BAN FORK" if is_fork(os.path.join(dbc, "car.capnp")) else "ban goc comma"
        print(f"-- {cer}")
        print(f"   car.capnp: {dbc}   [{tag}]")
        res, err = try_schema(cer, dbc, rlog)
        if err:
            print(f"   KHONG DOC DUOC: {err}\n")
            continue
        n, fp, sm, nd, nt = res
        print(f"   DOC DUOC: {n} ban tin | xe {fp} | safetyModel {sm} | "
              f"liveDelay {nd} | liveTracks {nt}")
        if sm.startswith("vinfast") and int(nd) > 0:
            print("   -> bo NAY dung: ten enum khop va khong mat ban tin nao\n")
            good.append((cer, dbc, True))
        else:
            print("   -> doc duoc nhung SAI TEN: safetyModel bao la "
                  f"'{sm}', liveDelay {nd} ban tin (dung phai > 0)\n")
            good.append((cer, dbc, False))

    best = next((g for g in good if g[2]), None)
    print("=" * 74)
    if best:
        cer, dbc, _ = best
        root = os.path.dirname(cer)
        print("DUNG BO NAY:\n")
        print(f"  export FORK={root}")
        print(f"  export OPENPILOT_DIR={root}")
        print(f"  export CEREAL_DIR={cer}")
        print(f"  export OPENDBC_DIR={dbc}")
        print("\nDan 4 dong tren vao terminal roi chay tiep cac script Phase.")
        return 0
    if good:
        cer, dbc, _ = good[0]
        print("CHI CO SCHEMA CUA BAN GOC — van chay duoc, nhung luu y:")
        print("  - moi con so do tu log deu DUNG")
        print("  - mot so TEN se sai (safetyModel bao 'volvo' thay vi 'vinfastVf6')")
        print("  - vai ban tin bi 'bien mat': liveDelay, liveTracks, liveParameters")
        print("  - nghia la Phase 1 muc 1.6 va Phase 4 muc 4.1 se thieu so lieu\n")
        print(f"  export CEREAL_DIR={cer}")
        print(f"  export OPENDBC_DIR={dbc}")
        print("\nMuon day du thi lay them 5 tep schema cua fork:")
        print(show_download())
        return 0
    print("Tim thay tep nhung khong bo nao doc duoc rlog nay.")
    print(show_download())
    return 1


def show_download():
    base = "https://raw.githubusercontent.com/qmpilot-vn/openpilot/vf-release-c4"
    return f"""
  mkdir -p ~/vf_schema/cereal/include ~/vf_schema/opendbc/car
  cd ~/vf_schema
  curl -fL {base}/cereal/log.capnp          -o cereal/log.capnp
  curl -fL {base}/cereal/custom.capnp       -o cereal/custom.capnp
  curl -fL {base}/cereal/deprecated.capnp   -o cereal/deprecated.capnp
  curl -fL {base}/cereal/include/c++.capnp  -o cereal/include/c++.capnp
  curl -fL {base}/opendbc_repo/opendbc/car/car.capnp -o opendbc/car/car.capnp
  ln -sf ../opendbc/car/car.capnp cereal/car.capnp

  export CEREAL_DIR=~/vf_schema/cereal
  export OPENDBC_DIR=~/vf_schema/opendbc/car

  (Neu repo la private thi curl se bao 404 — khi do copy tu chinh thiet bi tren xe:
     scp -r comma@<ip-xe>:/data/openpilot/cereal ~/vf_schema/cereal)
"""


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    root = os.path.expanduser(sys.argv[2]) if len(sys.argv) > 2 else None
    sys.exit(main(os.path.expanduser(sys.argv[1]), root))
