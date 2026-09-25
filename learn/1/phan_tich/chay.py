"""
chay.py — Chay tat ca trong mot lenh duy nhat.

    python3 chay.py

Khong can tham so gi. Script tu lam het:
  1. tim file rlog tren may
  2. tim schema capnp, thu doc rlog bang tung bo, chon bo tot nhat
  3. tim ma nguon openpilot de trich dan
  4. chay ca 5 Phase, luu ket qua vao thu muc ket_qua/
  5. in tom tat: cai gi chay duoc, cai gi thieu va vi sao

Tuy chon:
    python3 chay.py --rlog <duong_dan>     chi dinh san file rlog
    python3 chay.py --home <thu_muc>       quet o thu muc khac thay vi thu muc nha
    python3 chay.py --chi-tim              chi do tim, khong chay Phase nao
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BO_QUA = {".git", "__pycache__", "node_modules", ".cache", "Trash", ".venv", "venv", "site-packages"}
PHASES = [("phase1_modeld.py", "Phase 1 — modeld"),
          ("phase2_controls.py", "Phase 2 — controlsd"),
          ("phase3_mpc.py", "Phase 3 — MPC ga-phanh"),
          ("phase4_radard.py", "Phase 4 — radard"),
          ("phase5_selfdrived.py", "Phase 5 — selfdrived va panda")]


def quet(home, ten_tep, gioi_han=40):
    """Duyet thu muc nha tim mot ten tep, bo qua thung rac va cac thu muc rac."""
    found = []
    for root, dirs, files in os.walk(home):
        dirs[:] = [d for d in dirs if d not in BO_QUA and not d.startswith(".git")]
        if any(b in root for b in ("/Trash/", "/.cache/", "site-packages")):
            continue
        if ten_tep in files:
            found.append(os.path.join(root, ten_tep))
            if len(found) >= gioi_han:
                break
    return found


def do_giong(a, b):
    """Do do gan nhau cua hai duong dan — de ghep cereal voi car.capnp cung mot ban."""
    pa, pb = a.split(os.sep), b.split(os.sep)
    n = 0
    for x, y in zip(pa, pb):
        if x != y:
            break
        n += 1
    return n


def thu_doc(cereal, opendbc, rlog):
    """Thu doc rlog bang mot bo schema, chay trong tien trinh rieng de khong keo sap script."""
    code = (
        f"import sys; sys.path.insert(0, {HERE!r})\n"
        "import oplog\n"
        f"oplog.CEREAL={cereal!r}\noplog.OPENDBC={opendbc!r}\n"
        f"lg=oplog.Log({rlog!r})\ncp=lg.first('carParams')\n"
        "print(cp.carFingerprint, cp.safetyConfigs[0].safetyModel, "
        "len(lg.get('liveDelay')), len(lg.get('liveTracks')), len(lg.events))\n")
    try:
        r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=600)
    except subprocess.TimeoutExpired:
        return None
    if r.returncode != 0:
        return None
    return r.stdout.split()


def tim_ma_nguon(home):
    """Tim thu muc ma nguon openpilot — uu tien ban co ma VinFast."""
    ung_vien = []
    for p in quet(home, "controlsd.py", gioi_han=20):
        root = p.split(os.sep + "selfdrive")[0]
        try:
            co_vf = "vinfast" in open(p, encoding="utf-8", errors="replace").read().lower()
        except OSError:
            co_vf = False
        ung_vien.append((co_vf, root))
    ung_vien.sort(key=lambda x: not x[0])
    return ung_vien[0] if ung_vien else (False, None)


def main(argv):
    home = os.path.expanduser("~")
    rlog = None
    chi_tim = "--chi-tim" in argv
    for i, a in enumerate(argv):
        if a == "--home" and i + 1 < len(argv):
            home = os.path.expanduser(argv[i + 1])
        if a == "--rlog" and i + 1 < len(argv):
            rlog = os.path.expanduser(argv[i + 1])

    print("=" * 72)
    print("BUOC 1 — TIM FILE RLOG")
    print("=" * 72)
    if rlog is None:
        ds = [p for p in quet(home, "rlog.zst", 20)] or [p for p in quet(home, "rlog.bz2", 20)]
        if not ds:
            print(f"  Khong tim thay rlog nao trong {home}")
            print("  Chi dinh tay:  python3 chay.py --rlog <duong_dan>")
            return 1
        ds.sort(key=lambda p: (-os.path.getsize(p), p))
        rlog = ds[0]
        print(f"  Tim thay {len(ds)} file, dung file lon nhat:")
    print(f"  {rlog}")
    thu_muc_data = os.path.dirname(os.path.dirname(rlog))

    print("\n" + "=" * 72)
    print("BUOC 2 — TIM SCHEMA VA THU DOC")
    print("=" * 72)
    cereals = [os.path.dirname(p) for p in quet(home, "log.capnp", 20)]
    cars = [os.path.dirname(p) for p in quet(home, "car.capnp", 40)]
    if not cereals or not cars:
        print("  Khong tim thay log.capnp hoac car.capnp tren may.")
        print("  Xem muc 'Neu chua co schema nao' trong HUONG_DAN_CHAY.md")
        return 1

    ket_qua = []
    for cer in cereals:
        for dbc in sorted(cars, key=lambda d: -do_giong(cer, d))[:3]:
            r = thu_doc(cer, dbc, rlog)
            if r is None:
                continue
            fp, sm, nd, nt, ne = r
            diem = (2 if sm.startswith("vinfast") else 0) + (1 if int(nd) > 0 else 0)
            ket_qua.append((diem, cer, dbc, fp, sm, int(nd), int(nt), int(ne)))
            break                      # moi cereal chi can mot car.capnp doc duoc
    if not ket_qua:
        print("  Tim thay tep nhung khong bo nao doc duoc rlog nay.")
        return 1

    ket_qua.sort(key=lambda x: -x[0])
    for diem, cer, dbc, fp, sm, nd, nt, ne in ket_qua:
        dau = ">>" if (diem, cer) == (ket_qua[0][0], ket_qua[0][1]) else "  "
        print(f"  {dau} {cer}")
        print(f"       car.capnp: {dbc}")
        print(f"       {ne} ban tin | xe {fp} | safetyModel {sm} | liveDelay {nd} | liveTracks {nt}")
    diem, CER, DBC, fp, sm, nd, nt, ne = ket_qua[0]

    day_du = diem == 3
    if not day_du:
        print("\n  LUU Y: bo schema nay la cua BAN GOC comma, khong phai ban fork VinFast.")
        print("    - moi con so DO TU LOG deu dung")
        print(f"    - nhung mot so TEN sai: safetyModel hien la '{sm}', dung phai la 'vinfastVf6'")
        print(f"    - va {'liveDelay' if nd == 0 else ''} {'liveTracks' if nt == 0 else ''} bi doc thanh rong")
        print("    => Phase 1 muc 1.6 va Phase 4 muc 4.1 se thieu so lieu.")
        print("\n  Muon day du thi lay schema tu chinh xe (thay <ip-xe> bang dia chi thuc):")
        print("    scp -r comma@<ip-xe>:/data/openpilot/cereal ~/vf_schema_cereal")
        print("    scp comma@<ip-xe>:/data/openpilot/opendbc_repo/opendbc/car/car.capnp ~/vf_schema_cereal/")
        print("    python3 chay.py            # chay lai, no se tu nhan ra bo moi")

    print("\n" + "=" * 72)
    print("BUOC 3 — TIM MA NGUON DE TRICH DAN")
    print("=" * 72)
    co_vf, repo = tim_ma_nguon(home)
    if repo:
        print(f"  {repo}   [{'co ma VinFast' if co_vf else 'ban goc comma'}]")
    else:
        repo = "khong-co"
        print("  Khong tim thay — cac Phase van in du so lieu, chi bo qua phan trich ma nguon.")

    if chi_tim:
        print("\nDang o che do --chi-tim, khong chay Phase nao.")
        return 0

    print("\n" + "=" * 72)
    print("BUOC 4 — CHAY 5 PHASE")
    print("=" * 72)
    out_dir = os.path.join(os.getcwd(), "ket_qua")
    os.makedirs(out_dir, exist_ok=True)
    env = dict(os.environ, CEREAL_DIR=CER, OPENDBC_DIR=DBC, OPENPILOT_DIR=repo)
    loi = []
    for tep, ten in PHASES:
        dich = os.path.join(out_dir, tep.replace(".py", ".txt"))
        r = subprocess.run([sys.executable, os.path.join(HERE, tep), rlog, repo],
                           capture_output=True, text=True, env=env)
        open(dich, "w").write(r.stdout + ("\n--- LOI ---\n" + r.stderr if r.stderr else ""))
        n = len([l for l in r.stdout.splitlines() if l.strip()])
        if r.returncode == 0:
            print(f"  [OK]  {ten:32s} {n:4d} dong  ->  ket_qua/{os.path.basename(dich)}")
        else:
            loi.append(ten)
            print(f"  [LOI] {ten:32s} xem ket_qua/{os.path.basename(dich)}")

    print("\n" + "=" * 72)
    print("XONG")
    print("=" * 72)
    print(f"  Ket qua nam trong: {out_dir}")
    print(f"  Doc thu:  cat ket_qua/phase2_controls.txt")
    if loi:
        print(f"  Co {len(loi)} Phase loi: {', '.join(loi)}")
    if not day_du:
        print("  Nho: dang dung schema ban goc nen thieu mot phan so lieu (xem Buoc 2).")
    print("\n  Chay tiep tren toan bo route (lau hon, sinh file CSV):")
    print(f"    export OPENPILOT_DIR={repo}")
    print(f"    export CEREAL_DIR={CER}")
    print(f"    export OPENDBC_DIR={DBC}")
    print(f"    python3 {os.path.join(HERE,'selftest.py')} {rlog}")
    print(f"    python3 {os.path.join(HERE,'lane_analysis.py')} {thu_muc_data}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
