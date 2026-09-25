"""
long_analysis.py — Phan tich GA-PHANH va GIU KHOANG CACH (ACC) cua Openpilot.

Chay giong lane_analysis.py:

    python long_analysis.py ~/openpilot/data
    python long_analysis.py ~/route1 ~/route2 --max-seg 5

KET QUA (ghi ra thu muc hien tai)
---------------------------------
    L0_inventory.csv    moi segment 1 dong: co lead bao nhieu %, radar phu bao nhieu %,
                        so lan dap chan, suc khoe tin hieu aEgo
    L2_lead.csv         moi frame co xe phia truoc: thi giac doan bao nhieu met,
                        radar do bao nhieu met, sai so, xStd model tu bao
    L3_follow.csv       moi frame: khoang cach thoi gian (headway), thoi gian toi va cham (TTC)
    L4_comfort.csv      moi cua so 3 giay: lenh ga-phanh vs gia toc dat duoc, do giat
    L5_events.csv       moi lan tai xe dap ga/phanh de gianh quyen + dau hieu 3 giay truoc
    L6_envelope.csv     bang tong hop theo dieu kien

NGUYEN TAC DO
-------------
1. Khoang cach xe phia truoc: chi tin RADAR lam dap an (radarState.leadOne.radar == True,
   co radarTrackId that). Thi giac la thu duoc cham diem, khong duoc tu cham.
2. Gia toc thuc: dao ham toc do BANH XE (Savitzky-Golay). KHONG dung carState.aEgo:
   no bam DUNG HUONG (tuong quan 0.96) nhung SAI DO LON — nho hon thuc te khoang
   3.7 lan (cot aego_slope trong L0). Day la dau ra bo loc Kalman trong
   opendbc/car/interfaces.py:update_speed_kf, va longcontrol.py:87 lay chinh no
   lam sai so phan hoi: error = a_target - CS.aEgo.
3. Ghep frame: radarState.mdMonoTime tro thang ve modelV2.logMonoTime, khop tuyet doi,
   khong noi suy.
"""

import os
import sys
import warnings

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

from oplog import Log

warnings.filterwarnings("ignore", category=RuntimeWarning)

MIN_SPEED = 2.0          # m/s — duoi muc nay khong tinh headway/TTC
MIN_LEAD_PROB = 0.5      # nguong tin co xe phia truoc
SMOOTH_S = 0.5           # cua so lam muot khi tinh gia toc (giay)
WIN_S = 3.0              # cua so do do muot ga-phanh
WIN_STEP_S = 1.0
HARD_BRAKE = -2.0        # m/s^2
PRECURSOR_S = 3.0


# ══════════════════════════════════════════════════════════════════════════
# 1. GIA TOC THUC TU TOC DO BANH XE
# ══════════════════════════════════════════════════════════════════════════
def true_accel(t_ns, v):
    """Gia toc doc thuc = dao ham toc do BANH XE.

    Dung Savitzky-Golay tren luoi thoi gian deu: no khop da thuc bac 2 trong
    cua so roi lay dao ham giai tich, nen khong bi loi bien nhu cach lam muot
    roi gradient (cach do lam do giat TANG khi lam muot manh hon — vo ly).
    vEgo bi luong tu hoa 0.0156 m/s nen bat buoc phai lam muot; san nhieu do
    duoc chi ~0.05 m/s2, nho so voi tin hieu that ~0.28 m/s2.

    Tra (t_deu, gia_toc, do_giat) — tat ca tren luoi deu."""
    t = t_ns / 1e9
    dt = float(np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1], dt)
    vu = np.interp(tu, t, v)
    w_a = int(SMOOTH_S / dt) | 1          # 0.5s cho gia toc
    w_j = int(1.0 / dt) | 1               # 1.0s cho do giat (dao ham bac 2, nhieu hon)
    a = savgol_filter(vu, w_a, 2, deriv=1, delta=dt)
    j = savgol_filter(vu, w_j, 3, deriv=2, delta=dt)
    return tu, a, j


def lead_present(R):
    """Ban goc comma dat ten truong nay la 'present', ban fork dat la 'status'.
    Cung mot du lieu, chi khac ten — nen phai thu ca hai."""
    if hasattr(R, "present"):
        return bool(R.present)
    return bool(R.status)


def imu_long_axis(lg, t_ns, a_true):
    """Do truc doc cua gia toc ke bang cach doi chieu voi gia toc tu banh xe.
    Chi dung de KIEM TRA CHEO, khong dung lam dap an."""
    acc = lg.get("accelerometer")
    if len(acc) < 200:
        return None, np.nan
    at = np.array([e.logMonoTime for e in acc])
    best = (None, -9)
    for ax in range(3):
        av = np.array([e.accelerometer.acceleration.v[ax] for e in acc])
        for sg in (1, -1):
            c = np.corrcoef(np.interp(t_ns, at, sg * av), a_true)[0, 1]
            if np.isfinite(c) and c > best[1]:
                best = ((ax, sg), float(c))
    return best


# ══════════════════════════════════════════════════════════════════════════
# 2. DOC 1 SEGMENT
# ══════════════════════════════════════════════════════════════════════════
def read_segment(path, route, seg):
    lg = Log(path)
    cs_ev, cc_ev, mv_ev, rs_ev = (lg.get(n) for n in ("carState", "carControl", "modelV2", "radarState"))
    lp_ev = lg.get("longitudinalPlan")
    if not cs_ev or not cc_ev or not mv_ev or not rs_ev:
        return None, dict(route=route, segment=seg, ok=False, why="thieu ban tin co ban")

    cs_t = np.array([e.logMonoTime for e in cs_ev])
    v = np.array([e.carState.vEgo for e in cs_ev])
    t_u, a_u, j_u = true_accel(cs_t, v)
    a_true = np.interp(cs_t, t_u * 1e9, a_u)
    j_true = np.interp(cs_t, t_u * 1e9, j_u)
    a_can = np.array([e.carState.aEgo for e in cs_ev])
    gas = np.array([bool(e.carState.gasPressed) for e in cs_ev])
    brk = np.array([bool(e.carState.brakePressed) for e in cs_ev])
    standstill = np.array([bool(e.carState.standstill) for e in cs_ev])

    cc_t = np.array([e.logMonoTime for e in cc_ev])
    long_on = np.array([bool(e.carControl.longActive) for e in cc_ev])
    long_at_cs = np.interp(cs_t, cc_t, long_on.astype(float)) > 0.5

    imu, imu_corr = imu_long_axis(lg, cs_t, a_true)
    aego_corr = float(np.corrcoef(a_can, a_true)[0, 1]) if a_can.std() > 1e-6 else np.nan
    # He so nay moi la thu dang quan tam: aEgo co dung DO LON khong.
    aego_slope = float(np.polyfit(a_can, a_true, 1)[0]) if a_can.std() > 1e-6 else np.nan
    # san nhieu cua uoc luong gia toc: do tren cac doan xe chay deu
    steady = np.abs(np.interp(cs_t, t_u * 1e9, savgol_filter(np.interp(t_u, cs_t / 1e9, v), int(3.0 / 0.01) | 1, 2, deriv=1, delta=0.01))) < 0.05
    noise = float(a_true[steady].std()) if steady.sum() > 200 else np.nan
    cp = lg.first("carParams")
    fp = str(cp.carFingerprint) if cp is not None else "?"
    vf_patch = fp in ("VINFAST_VF8", "VINFAST_VF9")   # ban va phanh cua fork chi ap cho 2 xe nay

    dtc = np.diff(cs_t) / 1e9
    dur = float(np.sum(dtc[(dtc > 0) & (dtc < 1)]))
    km = float(np.sum(v[:-1] * np.clip(dtc, 0, 1)) / 1000)
    km_long = float(np.sum((v[:-1] * np.clip(dtc, 0, 1))[long_at_cs[:-1]]) / 1000)

    # ── ghep radar voi model qua mdMonoTime (khop tuyet doi) ──────────
    mv = {e.logMonoTime: e.modelV2 for e in mv_ev}
    lp_t = np.array([e.logMonoTime for e in lp_ev]) if lp_ev else np.array([])
    lp_src = [str(e.longitudinalPlan.longitudinalPlanSource) for e in lp_ev] if lp_ev else []
    lp_at = np.array([e.longitudinalPlan.aTarget for e in lp_ev]) if lp_ev else np.array([])

    rows, frows = [], []
    n_lead = n_radar = 0
    for e in rs_ev:
        R, t0 = e.radarState.leadOne, e.logMonoTime
        v0 = float(np.interp(t0, cs_t, v))
        on = bool(np.interp(t0, cc_t, long_on.astype(float)) > 0.5)
        if not lead_present(R):
            continue
        n_lead += 1
        has_radar = bool(R.radar) and R.radarTrackId != -1
        n_radar += has_radar
        m = mv.get(e.radarState.mdMonoTime)
        l = m.leadsV3[0] if (m is not None and len(m.leadsV3)) else None
        src = lp_src[int(np.argmin(np.abs(lp_t - t0)))] if len(lp_t) else ""

        # (a) cham diem thi giac bang radar
        if has_radar and l is not None and l.prob >= MIN_LEAD_PROB:
            rows.append(dict(route=route, segment=seg, t=t0, v_ego=v0, long_active=on,
                             d_radar=float(R.dRel), d_vision=float(l.x[0]),
                             err_d=abs(float(l.x[0]) - float(R.dRel)),
                             xstd=float(l.xStd[0]), prob=float(l.prob),
                             vrel_radar=float(R.vRel), vrel_vision=float(l.v[0]) - v0,
                             plan_src=src))
        # (b) hanh vi di theo — dung moi frame co lead
        if v0 >= MIN_SPEED:
            closing = -float(R.vRel)
            frows.append(dict(route=route, segment=seg, t=t0, v_ego=v0, long_active=on,
                              d_rel=float(R.dRel), co_radar=has_radar,
                              headway_s=float(R.dRel) / v0,
                              ttc_s=(float(R.dRel) / closing) if closing > 0.5 else np.nan,
                              v_lead=float(R.vLead), plan_src=src))

    lead = pd.DataFrame(rows)
    foll = pd.DataFrame(frows)

    # ── do muot ga-phanh theo cua so 3 giay ──────────────────────────
    wins = []
    t0 = cs_t[0]
    while t0 + int(WIN_S * 1e9) < cs_t[-1]:
        i0 = int(np.searchsorted(cs_t, t0))
        i1 = int(np.searchsorted(cs_t, t0 + int(WIN_S * 1e9)))
        t0 += int(WIN_STEP_S * 1e9)
        if i1 - i0 < 50 or v[i0:i1].min() < MIN_SPEED:
            continue
        on = long_at_cs[i0:i1]
        if on.mean() not in (0.0, 1.0):
            continue
        a = a_true[i0:i1]
        jerk = j_true[i0:i1]
        at_cmd = np.interp(cs_t[i0:i1], lp_t, lp_at) if len(lp_t) else np.full(i1 - i0, np.nan)
        srcs = [lp_src[int(np.argmin(np.abs(lp_t - tt)))] for tt in cs_t[i0:i1:20]] if len(lp_t) else []
        wins.append(dict(route=route, segment=seg, t=int(cs_t[i0]), long_active=bool(on.mean() == 1.0),
                         v_med_kmh=round(float(np.median(v[i0:i1])) * 3.6, 1),
                         a_med=round(float(np.median(a)), 4),
                         a_min=round(float(a.min()), 4),
                         jerk_std=round(float(np.std(jerk)), 4),
                         track_err_a=round(float(np.median(np.abs(at_cmd - a))), 4) if len(lp_t) else np.nan,
                         plan_src=max(set(srcs), key=srcs.count) if srcs else ""))
        # (khong reset t0 o day — da cong o tren)
    comfort = pd.DataFrame(wins)

    # ── su kien tai xe dap chan khi Openpilot dang giu ga ────────────
    ev_rows = []
    for sig, kind in ((gas, "dap_ga"), (brk, "dap_phanh")):
        idx = np.where((~sig[:-1]) & sig[1:])[0]
        for i in idx:
            te = cs_t[i]
            if not long_at_cs[i]:
                continue
            w = (foll.t >= te - int(PRECURSOR_S * 1e9)) & (foll.t <= te) if len(foll) else np.array([], bool)
            w = w.values if hasattr(w, "values") else w
            has = bool(np.any(w))
            ev_rows.append(dict(route=route, segment=seg, kind=kind, t=int(te),
                                v_kmh=round(float(np.interp(te, cs_t, v)) * 3.6, 1),
                                headway_s=round(float(foll.headway_s[w].median()), 2) if has else np.nan,
                                ttc_s=round(float(foll.ttc_s[w].min()), 2) if has else np.nan,
                                d_rel=round(float(foll.d_rel[w].median()), 1) if has else np.nan,
                                plan_src=foll.plan_src[w].mode()[0] if has and len(foll.plan_src[w].mode()) else "",
                                co_du_lieu_truoc=has))
    ev = pd.DataFrame(ev_rows)

    # ── diem dung: xe dang chay roi dung han ─────────────────────────
    # Do xem phanh bat dau som hay muon: so gia toc CAN THIET luc bat dau phanh
    # (v^2 / 2d) voi gia toc thuc su dat duoc. Phanh muon => phai phanh gap hon.
    stops = []
    for i in np.where((~standstill[:-1]) & standstill[1:])[0]:
        t_stop = cs_t[i]
        j0 = int(np.searchsorted(cs_t, t_stop - int(20 * 1e9)))
        seg_v, seg_a, seg_t = v[j0:i + 1], a_true[j0:i + 1], cs_t[j0:i + 1]
        if len(seg_v) < 100 or seg_v.max() < 5.0:
            continue                                  # bo qua cac lan bo cham roi dung
        k = int(np.where(seg_v >= 5.0)[0][-1])        # lan cuoi con chay tren 18 km/h
        br = np.where(seg_a[k:] < -0.3)[0]
        if not len(br):
            continue
        b = k + int(br[0])                            # thoi diem bat dau phanh
        v0 = float(seg_v[b])
        dt_ = np.diff(seg_t[b:] / 1e9)
        dist = float(np.sum(seg_v[b:-1] * np.clip(dt_, 0, 0.1)))
        if dist < 2.0:
            continue
        stops.append(dict(
            route=route, segment=seg, t=int(t_stop), long_active=bool(long_at_cs[b]),
            v_start_kmh=round(v0 * 3.6, 1), dist_m=round(dist, 1),
            time_s=round(float((seg_t[-1] - seg_t[b]) / 1e9), 1),
            a_can_thiet=round(-v0 ** 2 / (2 * dist), 3),      # gia toc trung binh bat buoc
            a_min=round(float(seg_a[b:].min()), 3),           # phanh gap nhat thuc te
            co_lead=bool(len(foll) and ((foll.t > t_stop - 5e9) & (foll.t <= t_stop)).any())))
    stops = pd.DataFrame(stops)

    inv = dict(route=route, segment=seg, ok=True, why="ok",
               duration_s=round(dur, 1), km=round(km, 3), km_long=round(km_long, 3),
               long_pct=round(100 * long_on.mean(), 1),
               v_med_kmh=round(float(np.median(v)) * 3.6, 1),
               lead_pct=round(100 * n_lead / max(len(rs_ev), 1), 1),
               radar_pct=round(100 * n_radar / max(n_lead, 1), 1),
               n_standstill=int(np.sum((~standstill[:-1]) & standstill[1:])),
               n_gas=int(np.sum((~gas[:-1]) & gas[1:])), n_brake=int(np.sum((~brk[:-1]) & brk[1:])),
               fingerprint=fp, vf_late_brake_patch=vf_patch,
               aego_corr=round(aego_corr, 3), aego_slope=round(aego_slope, 2),
               aego_std=round(float(a_can.std()), 4),
               a_true_std=round(float(a_true.std()), 4), a_noise=round(noise, 4) if noise == noise else np.nan,
               imu_axis=f"{imu[0]}/{imu[1]:+d}" if imu else "", imu_corr=round(imu_corr, 3))
    return (lead, foll, comfort, ev, stops), inv


# ══════════════════════════════════════════════════════════════════════════
# 3. BAO CAO
# ══════════════════════════════════════════════════════════════════════════
def lead_lag(lead, max_shift=20, step=2):
    """Uoc luong do tre giua thi giac va radar tren cac doan LIEN TUC.
    Tra (do_tre_giay, bang_sai_so_theo_do_lech)."""
    out = {}
    for k in range(-max_shift, max_shift + 1, step):
        es = []
        for (_, _), g in lead.groupby(["route", "segment"]):
            g = g.sort_values("t")
            d = np.diff(g.t.values) / 1e9
            brk = np.where(d > 0.06)[0]
            for i0, i1 in zip(np.r_[0, brk + 1], np.r_[brk + 1, len(g)]):
                a = g.d_vision.values[i0:i1]; b = g.d_radar.values[i0:i1]
                if len(a) < abs(k) + 20:
                    continue
                es += list(np.abs(a[max(k, 0):len(a) + min(k, 0)] - b[max(-k, 0):len(b) - max(k, 0)]))
        if len(es) > 50:
            out[k * 0.05] = float(np.median(es))
    if len(out) < 5:
        return np.nan, {}
    return min(out, key=out.get), out


def med(s):
    s = pd.Series(s).dropna()
    return float(s.median()) if len(s) else np.nan


def report(lead, foll, comfort, ev, stops, inv, outdir):
    g = inv[inv.ok == True] if "ok" in inv.columns else inv
    if not len(g) or "km_long" not in g.columns:
        print("\n" + "=" * 78)
        print("KHONG SEGMENT NAO DOC DUOC — khong co gi de bao cao.")
        print("=" * 78)
        if "why" in inv.columns and len(inv):
            from collections import Counter
            for ly_do, n in Counter(inv.why.astype(str)).most_common(5):
                print(f"  {n:3d} segment: {ly_do[:100]}")
        print("\nNeu ly do la \"no such member; name = present\" thi ban dang chay ban cu;")
        print("tai lai goi code moi nhat.")
        return
    km = g.km_long.sum()

    print("\n" + "=" * 78)
    print("TANG 0 — KIEM KE")
    print("=" * 78)
    print(f"Thoi gian {g.duration_s.sum()/60:.0f} phut | {g.km.sum():.1f} km | Openpilot giu ga {km:.1f} km ({g.long_pct.mean():.0f}% thoi gian)")
    print(f"Co xe phia truoc: {g.lead_pct.mean():.0f}% thoi gian | trong do RADAR nhin thay: {g.radar_pct.mean():.0f}%")
    print(f"So lan dung han: {int(g.n_standstill.sum())} | dap ga: {int(g.n_gas.sum())} | dap phanh: {int(g.n_brake.sum())}")
    print(f"\nXe: {g.fingerprint.mode()[0] if len(g.fingerprint.mode()) else '?'}"
          f" | ban va phanh VF8/VF9 cua fork co ap dung khong: {'CO' if g.vf_late_brake_patch.any() else 'KHONG'}")
    print(f"San nhieu cua uoc luong gia toc: {g.a_noise.median():.3f} m/s2 (tin hieu that {g.a_true_std.median():.3f})")
    print(f"Tin hieu aEgo tren CAN: tuong quan {g.aego_corr.median():.2f} nhung he so {g.aego_slope.median():.1f}x"
          f" (std {g.aego_std.median():.3f} vs thuc {g.a_true_std.median():.3f})")
    if g.aego_slope.median() > 1.5:
        print(f"  -> aEgo BAO NHO hon thuc te ~{g.aego_slope.median():.1f} lan. longcontrol.py:87 dung")
        print("     error = a_target - CS.aEgo lam phan hoi, nen vong dieu khien nhin thay")
        print("     minh giam toc it hon thuc te.")
    print(f"Kiem tra cheo bang IMU: tuong quan {g.imu_corr.median():.2f} (truc {g.imu_axis.mode()[0] if len(g.imu_axis.mode()) else '?'})")

    print("\n" + "=" * 78)
    print("TANG 2 — THI GIAC UOC LUONG KHOANG CACH XE TRUOC CO DUNG KHONG")
    print("=" * 78)
    if len(lead) < 100:
        print("Khong du mau.")
    else:
        r = float(np.corrcoef(lead.d_radar, lead.d_vision)[0, 1])
        slope = float(np.polyfit(lead.d_radar, lead.d_vision, 1)[0])
        print(f"Dap an = RADAR (n={len(lead)} frame co ca radar lan thi giac)")
        print(f"  tuong quan {r:+.3f} | he so {slope:.3f} "
              f"{'(thi giac doan NGAN hon thuc te)' if slope < 0.95 else '(thi giac doan XA hon thuc te)' if slope > 1.05 else '(khop)'}")
        bias = float((lead.d_vision - lead.d_radar).median())
        resid = (lead.d_vision - lead.d_radar - bias).abs()
        print(f"  sai so tong: median {med(lead.err_d):.2f} m | P90 {lead.err_d.quantile(.9):.2f} m "
              f"| tuong doi {100*med(lead.err_d/lead.d_radar):.1f}%")
        print(f"  TACH RA: lech co he thong {bias:+.2f} m (thi giac doan {'NGAN' if bias<0 else 'XA'} hon)")
        print(f"           do tan mac con lai {med(resid):.2f} m  <- day moi la sai so nhan thuc that")
        print(f"  (openpilot co hang so RADAR_TO_CAMERA = 1.52 m cho khoang cach radar->camera;")
        print(f"   lech do duoc gan bang con so do thi phan lon la quy doi he quy chieu, khong phai loi nhin)")
        lag, tab = lead_lag(lead)
        if tab:
            print(f"  Do tre thi giac so voi radar: {lag:+.2f}s "
                  f"(sai so nho nhat {min(tab.values()):.2f} m, tai lech 0: {tab.get(0.0, float('nan')):.2f} m)")
        ok = lead.xstd > 0
        print(f"  xStd model tu bao: median {med(lead.xstd[ok]):.2f} m | ty le sai-so/xStd = {med(lead.err_d[ok]/lead.xstd[ok]):.2f}")
        for lo, hi in ((0, 20), (20, 40), (40, 200)):
            d = lead[(lead.d_radar >= lo) & (lead.d_radar < hi)]
            if len(d) > 50:
                print(f"    {lo}-{hi}m: n={len(d):5d} sai so {med(d.err_d):.2f} m ({100*med(d.err_d/d.d_radar):.1f}%)")

    print("\n" + "=" * 78)
    print("TANG 3 — GIU KHOANG CACH CO AN TOAN KHONG")
    print("=" * 78)
    f = foll[foll.long_active] if len(foll) else foll
    if len(f) < 100:
        print("Khong du mau.")
    else:
        h = f.headway_s.dropna()
        print(f"Khoang cach thoi gian (headway) khi Openpilot giu ga, n={len(h)}:")
        print(f"  median {h.median():.2f}s | P10 {h.quantile(.1):.2f}s | duoi 1.0s: {100*(h<1).mean():.1f}% | duoi 0.6s: {100*(h<0.6).mean():.1f}%")
        tt = f.ttc_s.dropna()
        if len(tt) > 50:
            print(f"Thoi gian toi va cham (TTC) khi dang tien gan, n={len(tt)}:")
            print(f"  median {tt.median():.1f}s | P10 {tt.quantile(.1):.1f}s | duoi 3s: {100*(tt<3).mean():.1f}% | duoi 2s: {100*(tt<2).mean():.1f}%")
        hh = foll[~foll.long_active].headway_s.dropna()
        if len(hh) > 100:
            print(f"Doi chieu NGUOI lai: headway median {hh.median():.2f}s | duoi 1.0s: {100*(hh<1).mean():.1f}%")

    print("\n" + "=" * 78)
    print("TANG 4 — GA-PHANH CO MUOT KHONG")
    print("=" * 78)
    if len(comfort) < 30:
        print("Khong du mau.")
    else:
        print(f"{'nhom':22s} {'n':>6} {'giat (m/s3)':>12} {'phanh manh nhat':>16} {'bam lenh (m/s2)':>17}")
        for on in (True, False):
            d = comfort[comfort.long_active == on]
            if len(d) < 10:
                continue
            print(f"{'Openpilot' if on else 'Nguoi lai':22s} {len(d):>6} {med(d.jerk_std):>12.3f} "
                  f"{med(d.a_min):>16.3f} {med(d.track_err_a):>17.3f}")
        d = comfort[comfort.long_active]
        print(f"\nPhanh manh (duoi {HARD_BRAKE} m/s2): {100*(d.a_min < HARD_BRAKE).mean():.1f}% so cua so"
              f" = {(d.a_min < HARD_BRAKE).sum()/max(km,1e-9):.1f} lan/km")
        if d.plan_src.nunique() > 1:
            print("\nTheo nguon quyet dinh cua planner:")
            for src, dd in d.groupby("plan_src"):
                if len(dd) > 20:
                    print(f"  {src:12s} n={len(dd):5d} giat={med(dd.jerk_std):.3f} bam lenh={med(dd.track_err_a):.3f}")

    print("\n" + "=" * 78)
    print("TANG 5 — TAI XE DAP CHAN DE GIANH QUYEN")
    print("=" * 78)
    if not len(ev):
        print("Khong co su kien nao.")
    else:
        print(f"Tong {len(ev)} lan / {km:.1f} km = {len(ev)/max(km,1e-9):.1f} lan/km")
        for kind, d in ev.groupby("kind"):
            dd = d[d.co_du_lieu_truoc == True]
            print(f"\n-- {kind}: {len(d)} lan")
            if len(dd) >= 5:
                print(f"   luc do: toc do {med(dd.v_kmh):.0f} km/h | khoang cach {med(dd.d_rel):.0f} m "
                      f"| headway {med(dd.headway_s):.2f}s | TTC nho nhat {med(dd.ttc_s):.1f}s")
        base = foll[foll.long_active]
        if len(base) > 500 and len(ev[ev.co_du_lieu_truoc == True]) >= 10:
            d = ev[ev.co_du_lieu_truoc == True]
            rows = []
            for _, r in d.iterrows():
                b = base[(base.v_ego * 3.6 - r.v_kmh).abs() < 3]
                if len(b) < 200:
                    continue
                rows.append(dict(gan=r.headway_s < 1.0, nen_gan=(b.headway_s < 1.0).mean(),
                                 ttc=r.ttc_s < 3.0 if r.ttc_s == r.ttc_s else False,
                                 nen_ttc=(b.ttc_s < 3.0).mean()))
            m = pd.DataFrame(rows)
            if len(m) >= 10:
                print(f"\nDau hieu 3 giay truoc, so voi nen CUNG DAI TOC DO (+-3 km/h), n={len(m)}:")
                for a, b_, lbl in (("gan", "nen_gan", "headway < 1.0s"), ("ttc", "nen_ttc", "TTC < 3s")):
                    o, n = m[a].mean(), m[b_].mean()
                    print(f"  {lbl:16s} truoc can thiep {100*o:5.1f}%  nen {100*n:5.1f}%  = {o/max(n,1e-9):.2f}x")

    print("\n" + "=" * 78)
    print("TANG 7 — PHANH DUNG XE: som hay muon?")
    print("=" * 78)
    if len(stops) < 5:
        print("Khong du diem dung (can it nhat 5).")
    else:
        print(f"{'nhom':16s} {'n':>4} {'toc do bat dau':>15} {'quang duong':>12} {'a can thiet':>13} {'a thuc te':>11}")
        for on in (True, False):
            d = stops[stops.long_active == on]
            if len(d) < 3:
                continue
            print(f"{'Openpilot' if on else 'Nguoi lai':16s} {len(d):>4} {med(d.v_start_kmh):>14.1f}  "
                  f"{med(d.dist_m):>11.1f}m {med(d.a_can_thiet):>12.2f}  {med(d.a_min):>10.2f}")
        print("\n  a can thiet = v^2/2d, muc giam toc trung binh bat buoc tu luc bat dau phanh")
        print("  a thuc te   = luc phanh gap nhat da dung den")
        print("  Phanh cang muon => quang duong cang ngan => a can thiet cang am.")
        d = stops[stops.long_active]
        if len(d) >= 5:
            print(f"\n  Openpilot: {100*(d.a_min < -2.0).mean():.0f}% so lan dung phai phanh gap duoi -2 m/s2")

    # ── bang bien van hanh ────────────────────────────────────────────
    if len(foll) > 200:
        f = foll[foll.long_active].copy()
        f["vbin"] = pd.cut(f.v_ego * 3.6, [0, 30, 50, 200], labels=["<30", "30-50", ">50"])
        env = f.groupby(["vbin", "co_radar"], observed=True).agg(
            n=("headway_s", "size"), headway_med=("headway_s", "median"),
            headway_p10=("headway_s", lambda s: s.quantile(.1)),
            duoi_1s=("headway_s", lambda s: 100 * (s < 1).mean()),
            ttc_p10=("ttc_s", lambda s: s.quantile(.1))).reset_index()
        env["du_ket_luan"] = np.where(env.n >= 100, "co", "MAU QUA IT")
        env.to_csv(os.path.join(outdir, "L6_envelope.csv"), index=False)
        print("\n" + "=" * 78)
        print("TANG 6 — BIEN VAN HANH (chi luc Openpilot giu ga)")
        print("=" * 78)
        print(env.round(3).to_string(index=False))


# ══════════════════════════════════════════════════════════════════════════
def find_segments(paths):
    out = []
    for p in paths:
        p = os.path.abspath(os.path.expanduser(p))
        if not os.path.isdir(p):
            print(f"[bo qua] khong phai thu muc: {p}"); continue
        subs = sorted(os.listdir(p))
        has = any(s.startswith("segment_") and os.path.isdir(os.path.join(p, s)) for s in subs)
        routes = [(os.path.basename(p), p)] if has else [
            (s, os.path.join(p, s)) for s in subs if os.path.isdir(os.path.join(p, s))]
        for rname, rpath in routes:
            segs = [s for s in sorted(os.listdir(rpath))
                    if s.startswith("segment_") and os.path.isdir(os.path.join(rpath, s))]
            segs.sort(key=lambda s: int(s.split("_")[-1]) if s.split("_")[-1].isdigit() else 0)
            for s in segs:
                fn = next((os.path.join(rpath, s, n) for n in ("rlog.zst", "rlog.bz2", "rlog")
                           if os.path.isfile(os.path.join(rpath, s, n))), None)
                if fn:
                    out.append((rname, s, fn))
    return out


def main(paths, max_seg=None, outdir="."):
    segs = find_segments(paths)
    if max_seg:
        segs = segs[:max_seg]
    if not segs:
        print("Khong tim thay segment nao."); return
    print(f"Tim thay {len(segs)} segment tu {len({r for r,_,_ in segs})} route\n")

    L, F, C, E, S, I = [], [], [], [], [], []
    for n, (route, seg, fn) in enumerate(segs, 1):
        try:
            res, inv = read_segment(fn, route, seg)
        except Exception as ex:
            print(f"  [{n}/{len(segs)}] {route}/{seg}: LOI {type(ex).__name__}: {ex}")
            I.append(dict(route=route, segment=seg, ok=False, why=str(ex)))
            continue
        I.append(inv)
        if res is None:
            print(f"  [{n}/{len(segs)}] {route}/{seg}: bo qua — {inv.get('why')}")
            continue
        l, f, c, e, s_ = res
        L.append(l); F.append(f); C.append(c); E.append(e); S.append(s_)
        print(f"  [{n}/{len(segs)}] {route}/{seg}: {len(l)} frame co radar, {len(f)} frame co lead, {len(e)} su kien")

    cat = lambda xs: pd.concat(xs, ignore_index=True) if xs else pd.DataFrame()
    lead, foll, comfort, ev, stops, inv = cat(L), cat(F), cat(C), cat(E), cat(S), pd.DataFrame(I)
    for df, name in ((inv, "L0_inventory.csv"), (lead, "L2_lead.csv"), (foll, "L3_follow.csv"),
                     (comfort, "L4_comfort.csv"), (ev, "L5_events.csv"), (stops, "L7_stops.csv")):
        if len(df):
            df.to_csv(os.path.join(outdir, name), index=False)
    report(lead, foll, comfort, ev, stops, inv, outdir)
    print(f"\nDa ghi CSV vao: {os.path.abspath(outdir)}")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    mx = None
    for i, a in enumerate(sys.argv[1:]):
        if a.startswith("--max-seg"):
            mx = int(a.split("=")[1]) if "=" in a else int(sys.argv[i + 2])
    if not args:
        print(__doc__); sys.exit(1)
    main(args, max_seg=mx)
