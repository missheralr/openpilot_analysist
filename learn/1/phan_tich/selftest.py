"""
selftest.py — Kiem tra chinh lane_analysis.py truoc khi tin ket qua cua no.

    python selftest.py <duong_dan_1_segment/rlog.zst>

Chia lam 2 nhom:

  A. KIEM TRA TOAN HOC — khong can du lieu, dung quy dao tu che voi dap an biet truoc.
     Neu nhom nay sai thi cong thuc sai, khong phai du lieu sai.

  B. KIEM TRA TREN DU LIEU THAT — dung 1 segment. Quan trong nhat la phep thu
     DAU (test B1): neu dau cua "dap an" bi nguoc thi moi con so sai so deu vo nghia,
     va loi nay KHONG the phat hien bang cach nhin bang ket qua.

  C. THU PHA HOAI — co tinh lam hong du lieu de xem phep do co phat hien duoc khong.
     Mot phep do khong phan ung voi loi da gai la mot phep do vo dung.

Moi test in PASS hoac FAIL kem con so. Co FAIL thi dung, dung chay phan tich.
"""

import sys
import numpy as np

from oplog import Log
from lane_analysis import Truth, T_IDXS, H_IDX, HORIZONS_S, MIN_SPEED, WIN_S, model_curvature

PASS, FAIL = [], []


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:52s} {detail}")


def weave_amp(x, y):
    """Ban sao dung cong thuc trong lane_analysis: do lech so voi duong cong bac 3."""
    dev = y - np.polyval(np.polyfit(x, y, 3), x)
    return float(dev.max() - dev.min())


# ══════════════════════════════════════════════════════════════════════════
def nhom_A_toan_hoc():
    print("\nA. KIEM TRA TOAN HOC (quy dao tu che, dap an biet truoc)")

    # A1 — duong cong tron deu KHONG duoc tinh la danh vong
    v, dt = 15.0, 0.01
    t = np.arange(0, WIN_S, dt)
    R = 200.0
    th = v * t / R
    x, y = R * np.sin(th), R * (1 - np.cos(th))
    check("A1 cua tron deu -> danh vong ~ 0", weave_amp(x, y) < 0.02,
          f"do duoc {weave_amp(x, y)*100:.2f} cm (mong doi < 2 cm)")

    # A2 — danh vong hinh sin bien do biet truoc phai do ra dung
    A = 0.20                                    # bien do 0.2m -> dinh-day 0.4m
    xs = v * t
    ys = A * np.sin(2 * np.pi * t / 2.5)        # chu ky 2.5s, 2 chu ky trong cua so
    do = weave_amp(xs, ys)
    check("A2 danh vong sin 0.40m -> do ra dung", abs(do - 2 * A) < 0.08,
          f"do duoc {do:.3f} m (mong doi 0.400)")

    # A3 — danh vong tren duong cong: phai tach duoc dao dong khoi do cong
    xc, yc = x + 0, y + np.interp(x, xs, ys)
    do = weave_amp(xc, yc)
    check("A3 sin 0.40m tren duong cong -> van do ra dung", abs(do - 2 * A) < 0.08,
          f"do duoc {do:.3f} m (mong doi ~0.400)")

    # A4 — moc thoi gian T_IDXS dung chuan openpilot
    ok = (len(T_IDXS) == 33 and abs(T_IDXS[-1] - 10.0) < 1e-9
          and all(abs(T_IDXS[i] - HORIZONS_S[j]) < 0.35 for j, i in enumerate(H_IDX)))
    check("A4 moc thoi gian 1/2/3s khop luoi 33 diem", ok,
          f"chon {[round(T_IDXS[i],2) for i in H_IDX]} cho {HORIZONS_S}")


# ══════════════════════════════════════════════════════════════════════════
def nhom_B_du_lieu_that(path):
    print("\nB. KIEM TRA TREN DU LIEU THAT")
    lg = Log(path)
    tr = Truth(lg)
    if not tr.ok:
        check("B0 dung duoc dap an vat ly", False, tr.why)
        return None, None, None
    check("B0 dung duoc dap an vat ly", True, tr.why)

    st, v = lg.series("carState", "vEgo")
    my, ty, tv, dists = [], [], [], []
    for e in lg.get("modelV2"):
        t0, m = e.logMonoTime, e.modelV2
        if float(np.interp(t0, st, v)) < MIN_SPEED:
            continue
        a = tr.lateral_at_time(t0, T_IDXS[H_IDX[1]])      # moc 2 giay
        if a is None:
            continue
        my.append(float(m.position.y[H_IDX[1]]))
        ty.append(a)
        tv.append(float(np.interp(t0, st, v)))
        x, y, _ = tr.path_from(t0, 12.0)
        dists.append(x[-1] if x is not None else np.nan)
    my, ty = np.array(my), np.array(ty)
    if len(my) < 100:
        check("B1 du mau de kiem tra", False, f"chi co {len(my)}")
        return tr, my, ty

    # B1 — DAU. Day la phep thu quan trong nhat.
    r = float(np.corrcoef(my, ty)[0, 1])
    check("B1 dau cua dap an khop voi model (tuong quan > 0.7)", r > 0.7,
          f"r = {r:+.3f}  (am = dap an bi lat trai/phai)")

    # B2 — do lon. Neu he so ty le lech xa 1 thi mot ben sai don vi hoac sai he quy chieu.
    slope = float(np.polyfit(ty, my, 1)[0])
    check("B2 ty le do lon ~ 1 (0.5 - 1.5)", 0.5 < slope < 1.5,
          f"he so = {slope:.3f}")

    # B3 — sai so phai tang dan theo tam nhin
    errs = []
    for j, h in zip(H_IDX, HORIZONS_S):
        e = []
        for ev in lg.get("modelV2"):
            t0 = ev.logMonoTime
            if float(np.interp(t0, st, v)) < MIN_SPEED:
                continue
            a = tr.lateral_at_time(t0, T_IDXS[j])
            if a is not None:
                e.append(abs(float(ev.modelV2.position.y[j]) - a))
        errs.append(np.median(e) if e else np.nan)
    check("B3 sai so tang dan 1s < 2s < 3s", errs[0] < errs[1] < errs[2],
          f"{errs[0]:.3f} < {errs[1]:.3f} < {errs[2]:.3f} m")

    # B4 — quang duong IMU tich phan phai khop dong ho odo tu toc do banh xe
    span = 20.0
    t0 = tr.t[len(tr.t) // 4]
    x, y, tt = tr.path_from(t0, span)
    if x is not None:
        i0 = int(np.searchsorted(tr.t, t0))
        i1 = i0 + len(x)
        dt_ = np.diff(tr.t[i0:i1]) / 1e9
        s_odo = float(np.sum(tr.v[i0:i1 - 1] * np.clip(dt_, 0, 0.1)))
        s_path = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
        check("B4 chieu dai duong di khop odo banh xe (<1%)",
              abs(s_path - s_odo) / max(s_odo, 1e-9) < 0.01,
              f"duong di {s_path:.1f}m vs odo {s_odo:.1f}m")

    # B5 — cua so 5 giay luon nam gon trong vung du lieu lien tuc
    d = np.array(dists)
    bad = np.where(~np.isfinite(d))[0]
    ok = len(bad) == 0 or bad.min() > len(d) - 12 * 20 - 5   # chi o ~12s cuoi
    check("B5 thieu du lieu chi xay ra o cuoi segment", ok,
          f"{len(bad)}/{len(d)} diem thieu, som nhat o vi tri {bad.min() if len(bad) else '-'}")
    return tr, my, ty


# ══════════════════════════════════════════════════════════════════════════
def nhom_C_pha_hoai(tr, my, ty):
    print("\nC. THU PHA HOAI (gai loi xem phep do co bat duoc khong)")
    if my is None or len(my) < 100:
        print("  (bo qua — khong du mau)")
        return
    base = float(np.median(np.abs(my - ty)))
    print(f"  sai so goc = {base:.3f} m")

    # C1 — day lech dap an 0.5m: sai so phai tang gan dung 0.5m
    e = float(np.median(np.abs(my - (ty + 0.5))))
    check("C1 gai lech 0.5m -> sai so tang tuong ung", e > base + 0.30,
          f"{base:.3f} -> {e:.3f} m")

    # C2 — lat dau dap an: sai so phai tang manh (neu KHONG tang, phep do bi diec)
    e = float(np.median(np.abs(my + ty)))
    check("C2 gai lat dau trai/phai -> sai so tang manh", e > base * 1.5,
          f"{base:.3f} -> {e:.3f} m")

    # C3 — xao tron thu tu: sai so phai tang manh (chung minh ghep dung frame)
    rng = np.random.default_rng(0)
    e = float(np.median(np.abs(my - rng.permutation(ty))))
    check("C3 gai xao tron frame -> sai so tang manh", e > base * 1.5,
          f"{base:.3f} -> {e:.3f} m")

    # C4 — lech thoi gian 1 giay: sai so phai tang (chung minh dong bo thoi gian dung)
    k = 20
    e = float(np.median(np.abs(my[:-k] - ty[k:])))
    check("C4 gai lech thoi gian 1s -> sai so tang", e > base,
          f"{base:.3f} -> {e:.3f} m")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Dung: python selftest.py <rlog.zst>")
        sys.exit(1)
    nhom_A_toan_hoc()
    tr, my, ty = nhom_B_du_lieu_that(sys.argv[1])
    nhom_C_pha_hoai(tr, my, ty)
    print(f"\n{'='*70}\nKET QUA: {len(PASS)} PASS, {len(FAIL)} FAIL")
    if FAIL:
        print("Chua duoc chay phan tich. Cac test hong:")
        for f in FAIL:
            print("  - " + f)
        sys.exit(1)
    print("Tat ca dat. Chay lane_analysis.py duoc.")
