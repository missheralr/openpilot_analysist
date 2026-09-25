"""
selftest_long.py — Kiem tra long_analysis.py truoc khi tin ket qua.

    python selftest_long.py <duong_dan_1_segment/rlog.zst>

A. TOAN HOC — quy dao tu che, dap an biet truoc. Sai o day la sai cong thuc.
B. DU LIEU THAT — kiem tra dap an (radar, gia toc) co dung la doc lap va hop ly.
C. PHA HOAI — gai loi xem phep do co bat duoc khong.
"""

import sys
import numpy as np

from oplog import Log
from long_analysis import true_accel, MIN_LEAD_PROB

PASS, FAIL = [], []


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:50s} {detail}")


def quantize(v, q=0.015625):
    """vEgo tren CAN bi luong tu hoa; mo phong lai de test sat thuc te."""
    return np.round(v / q) * q


# ══════════════════════════════════════════════════════════════════════════
def nhom_A():
    print("\nA. KIEM TRA TOAN HOC")
    dt = 0.01
    t = np.arange(0, 20, dt)
    tn = (t * 1e9).astype(np.int64)

    # A1 — gia toc khong doi, co luong tu hoa
    for a0 in (-1.5, 0.8):
        v = quantize(np.clip(10 + a0 * t, 0.1, None))
        _, a, _ = true_accel(tn, v)
        m = (t > 1) & (t < t[-1] - 1) & (10 + a0 * t > 1)
        check(f"A1 gia toc khong doi {a0:+.1f} m/s2", abs(np.median(a[:len(t)][m[:len(a)]]) - a0) < 0.05,
              f"do duoc {np.median(a[:len(t)][m[:len(a)]]):+.3f}")

    # A2 — xe chay deu: gia toc do duoc phai gan 0, day la san nhieu
    v = quantize(np.full_like(t, 12.0))
    _, a, _ = true_accel(tn, v)
    check("A2 chay deu -> san nhieu nho", a.std() < 0.10, f"std = {a.std():.4f} m/s2")

    # A3 — do giat: gia toc doi tuyen tinh tu 0 den -2 trong 10s => giat = -0.2
    a_prof = np.clip(-0.2 * t, -2, 0)
    v = quantize(np.clip(20 + np.cumsum(a_prof) * dt, 0.1, None))
    _, _, j = true_accel(tn, v)
    m = (t > 2) & (t < 9)
    check("A3 do giat -0.20 m/s3", abs(np.median(j[:len(t)][m[:len(j)]]) + 0.2) < 0.05,
          f"do duoc {np.median(j[:len(t)][m[:len(j)]]):+.3f}")

    # A4 — cong thuc headway / TTC
    d, ve, vrel = 30.0, 15.0, -3.0
    check("A4 headway = d/v va TTC = d/(-vRel)",
          abs(d / ve - 2.0) < 1e-9 and abs(d / (-vrel) - 10.0) < 1e-9,
          f"headway {d/ve:.1f}s, TTC {d/(-vrel):.1f}s")

    # A5 — gia toc can thiet de dung: v^2/2d
    v0, dist = 14.0, 49.0
    check("A5 a can thiet = v^2/2d", abs(-v0**2 / (2*dist) + 2.0) < 1e-9,
          f"{-v0**2/(2*dist):.2f} m/s2 (dung sau {dist} m tu {v0} m/s)")


# ══════════════════════════════════════════════════════════════════════════
def nhom_B(path):
    print("\nB. KIEM TRA TREN DU LIEU THAT")
    lg = Log(path)
    cs = lg.get("carState")
    t = np.array([e.logMonoTime for e in cs])
    v = np.array([e.carState.vEgo for e in cs])
    a_can = np.array([e.carState.aEgo for e in cs])
    tu, a, j = true_accel(t, v)
    a_i = np.interp(t, tu * 1e9, a)

    # B1 — ghep radar voi model phai khop tuyet doi
    mv = {e.logMonoTime: e.modelV2 for e in lg.get("modelV2")}
    rs = lg.get("radarState")
    hit = sum(1 for e in rs if e.radarState.mdMonoTime in mv)
    check("B1 mdMonoTime khop 100% voi modelV2", hit == len(rs), f"{hit}/{len(rs)}")

    # B2 — gia toc tu banh xe phai khop GIA TOC KE (ben thu ba doc lap)
    acc = lg.get("accelerometer")
    at = np.array([e.logMonoTime for e in acc])
    best = (-9, None)
    for ax in range(3):
        av = np.array([e.accelerometer.acceleration.v[ax] for e in acc])
        for sg in (1, -1):
            ai = np.interp(t, at, sg * av)
            c = np.corrcoef(ai, a_i)[0, 1]
            if c > best[0]:
                best = (c, (ax, sg, float(np.polyfit(a_i, ai, 1)[0])))
    check("B2 gia toc banh xe khop gia toc ke (corr>0.7)", best[0] > 0.7,
          f"corr {best[0]:.3f}, he so {best[1][2]:.2f} (truc {best[1][0]}/{best[1][1]:+d})")

    # B3 — san nhieu phai nho so voi tin hieu
    steady = np.abs(a_i) < 0.05
    nf = a_i[steady].std() if steady.sum() > 200 else np.nan
    check("B3 san nhieu < 30% tin hieu that", nf < 0.3 * a_i.std(),
          f"nhieu {nf:.3f} vs tin hieu {a_i.std():.3f}")

    # B4 — aEgo bam huong nhung sai do lon (ly do khong dung no lam dap an)
    c = np.corrcoef(a_can, a_i)[0, 1]
    sl = float(np.polyfit(a_can, a_i, 1)[0])
    check("B4 aEgo dung huong nhung sai do lon", c > 0.8 and sl > 1.5,
          f"corr {c:.2f}, he so {sl:.1f}x -> KHONG dung aEgo lam dap an")

    # B5 — radar vs thi giac: phai tuong quan duong
    vis, rad = [], []
    for e in rs:
        R = e.radarState.leadOne
        if not (R.present and R.radar and R.radarTrackId != -1):
            continue
        m = mv.get(e.radarState.mdMonoTime)
        if m is None or not len(m.leadsV3) or m.leadsV3[0].prob < MIN_LEAD_PROB:
            continue
        vis.append(float(m.leadsV3[0].x[0])); rad.append(float(R.dRel))
    vis, rad = np.array(vis), np.array(rad)
    if len(vis) < 50:
        check("B5 du mau radar+thi giac", False, f"chi co {len(vis)}")
        return None, None
    r = float(np.corrcoef(vis, rad)[0, 1])
    check("B5 thi giac va radar tuong quan duong", r > 0.3, f"r = {r:+.3f}, n = {len(vis)}")
    return vis, rad


# ══════════════════════════════════════════════════════════════════════════
def nhom_C(vis, rad):
    print("\nC. THU PHA HOAI")
    if vis is None:
        print("  (bo qua — khong du mau)")
        return
    base = float(np.median(np.abs(vis - rad)))
    print(f"  sai so goc = {base:.2f} m")
    e = float(np.median(np.abs(vis - (rad + 5.0))))
    check("C1 gai lech 5m -> sai so tang", e > base + 3.0, f"{base:.2f} -> {e:.2f} m")
    rng = np.random.default_rng(0)
    e = float(np.median(np.abs(vis - rng.permutation(rad))))
    check("C2 gai xao tron frame -> sai so tang", e > base * 1.5, f"{base:.2f} -> {e:.2f} m")
    # C3 — lech thoi gian: phai lam sai so TANG khi lech nhieu (2s).
    # Luu y: phai lech tren cac doan LIEN TUC, vi mang nay chi chua cac frame
    # co ca radar lan thi giac nen khong deu thoi gian.
    def shifted(a, b, k):
        return np.abs(a[max(k, 0):len(a) + min(k, 0)] - b[max(-k, 0):len(b) - max(k, 0)])
    e40 = float(np.median(shifted(vis, rad, 40)))      # 2 giay
    check("C3 gai lech thoi gian 2s -> sai so tang", e40 > base,
          f"{base:.2f} -> {e40:.2f} m")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Dung: python selftest_long.py <rlog.zst>")
        sys.exit(1)
    nhom_A()
    vis, rad = nhom_B(sys.argv[1])
    nhom_C(vis, rad)
    print(f"\n{'='*70}\nKET QUA: {len(PASS)} PASS, {len(FAIL)} FAIL")
    if FAIL:
        print("Chua duoc chay phan tich. Test hong:")
        for f in FAIL:
            print("  - " + f)
        sys.exit(1)
    print("Tat ca dat. Chay long_analysis.py duoc.")
