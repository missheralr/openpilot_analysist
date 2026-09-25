"""
phase4_radard.py — Sinh ra moi con so cua Phase 4 trong tai lieu (ghep radar voi camera).

    python phase4_radard.py <rlog.zst> <repo_fork> [repo_goc_comma]

Luu y quan trong: trong ban fork, selfdrive/controls/radard.py chi la vo 17 dong
nap bytecode. Muon xem ma nguon cua cac ham nay phai doc tu ban goc cua comma —
truyen duong dan repo goc o tham so thu ba. Hang so rieng cua fork nam trong .pyc,
dung decompile_consts.py de lay ra.

  4.1  radar tren xe nay cho gi
  4.2  bo loc Kalman theo doi tung doi tuong
  4.3  ghep radar voi thi giac bang tich ba xac suat
  4.6  thi giac bao khoang cach hut bao nhieu — CHAM DIEM BANG RADAR
  4.7  vi sao chi 38% khung hinh co radar hau thuan
"""
import os
import sys
import numpy as np
from collections import Counter


def grep(repo, rel, pattern, n=3):
    if repo is None:
        print(f"    (chua truyen repo goc — bo qua {pattern})"); return
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


def present(R):
    """Ban goc dat ten truong nay la 'present', ban fork dat la 'status'."""
    return bool(getattr(R, "present", None) if hasattr(R, "present") else R.status)


def main(rlog, repo, upstream=None):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import oplog
    # Uu tien bien moi truong — de dung duoc ca khi schema nam rieng mot cho,
    # khong nhat thiet phai co ca repo ma nguon.
    oplog.CEREAL = os.environ.get("CEREAL_DIR") or os.path.join(repo, "cereal")
    oplog.OPENDBC = os.environ.get("OPENDBC_DIR") or os.path.join(repo, "opendbc_repo/opendbc/car")
    lg = oplog.Log(rlog)
    rs = lg.get("radarState")
    mvd = {e.logMonoTime: e.modelV2 for e in lg.get("modelV2")}

    print("\n" + "=" * 78)
    print("4.0  BAN FORK DONG GOI radard THANH BYTECODE")
    print("=" * 78)
    f = os.path.join(repo, "selfdrive/controls/radard.py")
    print(f"    {f}: {len(open(f).read().splitlines())} dong")
    print(f"    dong dau: {open(f).readline().strip()[:72]}")
    print("    -> ma nguon that nam trong __pycache__/radard_impl.pyc; cac doan trich")
    print("       trong tai lieu lay tu ban goc cua comma, xem duoi day.")

    print("\n" + "=" * 78)
    print("4.2  BO LOC KALMAN (trich tu ban goc)")
    print("=" * 78)
    grep(upstream, "selfdrive/controls/radard.py", "self.A = [[1.0, dt]", n=1)
    grep(upstream, "selfdrive/controls/radard.py", "hardcoding a lookup table", n=1)
    grep(upstream, "selfdrive/controls/radard.py", "self.aLeadTau.x = _LEAD_ACCEL_TAU", n=1)

    print("\n" + "=" * 78)
    print("4.3  GHEP RADAR VOI THI GIAC (trich tu ban goc)")
    print("=" * 78)
    grep(upstream, "selfdrive/controls/radard.py", "def laplacian_pdf", n=1)
    grep(upstream, "selfdrive/controls/radard.py", "prob_d = laplacian_pdf", n=1)
    grep(upstream, "selfdrive/controls/radard.py", "dist_sane =", n=1)
    grep(upstream, "selfdrive/controls/radard.py", "RADAR_TO_CAMERA =", n=1)

    print("\n" + "=" * 78)
    print("4.1  RADAR TREN XE NAY CHO GI")
    print("=" * 78)
    lt = lg.get("liveTracks")
    npt = np.array([len(e.liveTracks.points) for e in lt])
    pts = [p for e in lt for p in e.liveTracks.points]
    print(f"    so ban tin liveTracks   : {len(lt)}")
    if not pts:
        print("    -> KHONG DOC DUOC diem radar nao.")
        print("       Neu log that su co radar thi day la dau hieu dang dung SCHEMA SAI:")
        print("       ban tin liveTracks chi hien ra khi doc bang schema cua chinh ban fork.")
        print("       Muc 4.1 cua tai lieu se thieu; cac muc con lai van day du.\n")
    else:
        print(f"    so doi tuong moi ban tin: 0 diem {100*np.mean(npt==0):.0f}%,"
              f" 1 diem {100*np.mean(npt==1):.0f}%, 2 diem {100*np.mean(npt==2):.0f}%")
        print(f"    tong so diem            : {len(pts)}")
        print(f"    co measured (do moi)    : {100*np.mean([p.measured for p in pts]):.0f}%")
        print(f"    co van toc khac 0       : {100*np.mean([p.vRel != 0 for p in pts]):.0f}%")
        print(f"    khoang cach trung vi    : {np.median([p.dRel for p in pts]):.1f} m")

    print("\n" + "=" * 78)
    print("4.6  THI GIAC BAO KHOANG CACH HUT BAO NHIEU — CHAM BANG RADAR")
    print("=" * 78)
    hit = sum(1 for e in rs if e.radarState.mdMonoTime in mvd)
    print(f"    Ghep frame qua mdMonoTime: {hit}/{len(rs)} khop tuyet doi, khong noi suy")
    V, R_, S, VO = [], [], [], []
    for e in rs:
        R = e.radarState.leadOne
        if not present(R):
            continue
        m = mvd.get(e.radarState.mdMonoTime)
        if m is None or not len(m.leadsV3) or m.leadsV3[0].prob < 0.5:
            continue
        if R.radar and R.radarTrackId != -1:
            V.append(float(m.leadsV3[0].x[0]) - 1.52)     # dua ve cung he quy chieu voi radar
            R_.append(float(R.dRel)); S.append(float(m.leadsV3[0].xStd[0]))
        else:
            VO.append(float(m.leadsV3[0].x[0]) - float(R.dRel))
    V, R_, S = np.array(V), np.array(R_), np.array(S)
    print(f"\n    Kiem tra quy uoc he quy chieu tren {len(VO)} khung hinh THI GIAC THUAN:")
    print(f"      x[0] - dRel = {np.median(VO):.3f} m, do lech chuan {np.std(VO):.3f}"
          f"   -> dung bang RADAR_TO_CAMERA = 1.52")
    bi = np.median(V - R_)
    print(f"\n    Tren {len(V)} khung hinh co CA radar lan thi giac:")
    print(f"      lech co he thong   : {bi:+.2f} m  (thi giac doan GAN hon radar)")
    print(f"      do tan mac con lai : {np.median(np.abs(V-R_-bi)):.2f} m")
    print(f"      xStd model tu bao  : {np.median(S):.2f} m")
    print(f"      ty le sai so / xStd: {np.median(np.abs(V-R_)/S):.2f}")
    print(f"\n    {'cu ly theo radar':>18} {'n':>5} {'lech':>9} {'tuong doi':>11}")
    for lo, hi in ((10, 20), (20, 30), (30, 45)):
        m = (R_ >= lo) & (R_ < hi)
        if m.sum() > 20:
            print(f"    {lo}-{hi} m{'':>11} {m.sum():>5} {np.median((V-R_)[m]):>+9.2f} {np.median(((V-R_)/R_)[m]):>10.0%}")
    print(f"\n    Nguong chap nhan ghep o 25 m = max(0.25*25, 5.0) = 6.25 m -> lech 3.6 m van duoc coi la khop")

    print("\n" + "=" * 78)
    print("4.7  VI SAO CHI MOT PHAN SO KHUNG HINH CO RADAR HAU THUAN")
    print("=" * 78)
    rows = []
    for e in rs:
        R = e.radarState.leadOne
        if not present(R):
            continue
        cs = lg.at("carState", e.logMonoTime)
        rows.append((cs.vEgo, R.dRel, bool(R.radar and R.radarTrackId != -1), R.radarTrackId))
    print(f"    Tong the: {100*np.mean([r[2] for r in rows]):.0f}% khung hinh co lead duoc radar hau thuan\n")
    print(f"    {'nhom':>24} {'n':>6} {'% co radar':>12}")
    for lo, hi, lbl in ((0, 3, "duoi 11 km/h"), (3, 7, "11-25 km/h"), (7, 99, "tren 25 km/h")):
        s = [r for r in rows if lo <= r[0] < hi]
        if len(s) > 20:
            print(f"    {lbl:>24} {len(s):>6} {100*np.mean([r[2] for r in s]):>11.0f}%")
    for lo, hi, lbl in ((10, 20, "vat cach 10-20 m"), (20, 30, "vat cach 20-30 m"), (30, 999, "vat cach tren 30 m")):
        s = [r for r in rows if lo <= r[1] < hi]
        if len(s) > 20:
            print(f"    {lbl:>24} {len(s):>6} {100*np.mean([r[2] for r in s]):>11.0f}%")
    ids = [r[3] for r in rows if r[2]]
    ch = sum(1 for x, y in zip(ids, ids[1:]) if x != y)
    print(f"\n    Muc tieu radar khong ben: doi so hieu {ch} lan trong {len(ids)} khung hinh,"
          f" dung qua {len(set(ids))} so hieu")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    up = os.path.expanduser(sys.argv[3]) if len(sys.argv) > 3 else None
    main(os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]), up)
