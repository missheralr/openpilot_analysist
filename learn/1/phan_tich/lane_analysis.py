"""
lane_analysis.py — Phân tích LANE PREDICTION và LANE KEEPING của Openpilot.

Chạy trên 1 hoặc nhiều route, tự động toàn bộ: kiểm kê dữ liệu, dựng đáp án vật lý,
đo chất lượng dự đoán quỹ đạo, đo chất lượng giữ làn, tìm sự kiện người can thiệp,
và tổng hợp thành bản đồ biên vận hành.

CÁCH CHẠY
---------
    python lane_analysis.py ~/openpilot/data                 # 1 route
    python lane_analysis.py ~/openpilot/data ~/route2 ...    # nhiều route
    python lane_analysis.py ~/all_routes                     # thư mục chứa nhiều route
    python lane_analysis.py ~/openpilot/data --max-seg 3     # chạy thử nhanh

KẾT QUẢ (ghi ra thư mục hiện tại)
---------------------------------
    00_inventory.csv    mỗi segment 1 dòng: dữ liệu có gì, chất lượng ra sao
    02_prediction.csv   mỗi frame 1 dòng: sai số dự đoán quỹ đạo + điều kiện
    03_keeping.csv      mỗi cửa sổ 5 giây 1 dòng: chỉ số giữ làn
    04_events.csv       mỗi lần người can thiệp 1 dòng + dấu hiệu 3 giây trước
    05_envelope.csv     bảng tổng hợp theo điều kiện
    05_clips.csv        danh sách đoạn tệ nhất, để xem lại bằng video

NGUYÊN TẮC: đáp án (xe thực sự đi đâu) dựng từ CẢM BIẾN VẬT LÝ — tốc độ bánh xe
và con quay IMU. KHÔNG dùng cameraOdometry vì đó là đầu ra của chính model lái.
"""

import os
import sys
import math
import warnings

import numpy as np
import pandas as pd

from oplog import Log

warnings.filterwarnings("ignore", category=RuntimeWarning)

# ── Tham số ───────────────────────────────────────────────────────────────
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
HORIZONS_S = [1.0, 2.0, 3.0]                  # dự đoán theo thời gian
HORIZONS_M = [10.0, 20.0, 30.0]               # dự đoán theo quãng đường
H_IDX = [int(np.argmin(np.abs(T_IDXS - h))) for h in HORIZONS_S]
MIN_SPEED = 2.0                               # m/s — dưới mức này bỏ qua
WIN_S = 5.0                                   # cửa sổ đo giữ làn
WIN_STEP_S = 1.0
WIN_MIN_SPEED = 3.0
STRAIGHT_K = 0.002                            # |độ cong| dưới mức này coi là đường thẳng
LANE_PROB_OK = 0.3
MIN_IMU_CORR = 0.9
PRECURSOR_S = 3.0                             # cửa sổ tìm dấu hiệu trước sự kiện


# ═══════════════════════════════════════════════════════════════════════════
# 1. ĐÁP ÁN VẬT LÝ: tốc độ bánh xe + con quay IMU
# ═══════════════════════════════════════════════════════════════════════════
class Truth:
    """Dựng lại quỹ đạo xe thực sự đã đi, độc lập hoàn toàn với camera/model."""

    def __init__(self, lg):
        self.ok, self.why = False, ""
        g = lg.get("gyroscope")
        if len(g) < 200:
            self.why = "thieu gyroscope"
            return
        self.t = np.array([e.logMonoTime for e in g], dtype=np.int64)
        gv = np.array([list(e.gyroscope.gyroUncalibrated.v) for e in g])

        st, v = lg.series("carState", "vEgo")
        kt, kk = lg.series("controlsState", "curvature")
        if len(st) < 100 or len(kt) < 100:
            self.why = "thieu carState/controlsState"
            return
        self.v = np.interp(self.t, st, v)
        self.k = np.interp(self.t, kt, kk)
        yaw_ref = self.v * self.k                       # tốc độ quay suy từ góc vô-lăng

        moving = self.v > MIN_SPEED
        if moving.sum() < 200:
            self.why = "xe gan nhu dung yen"
            return
        # tự dò trục và dấu của con quay (cách lắp thiết bị có thể khác nhau)
        cand = [(ax, sg, np.corrcoef(sg * gv[moving, ax], yaw_ref[moving])[0, 1])
                for ax in range(3) for sg in (1, -1)]
        ax, sg, corr = max(cand, key=lambda z: (z[2] if np.isfinite(z[2]) else -9))
        self.imu_axis, self.imu_sign, self.imu_corr = ax, sg, float(corr)
        if not np.isfinite(corr) or corr < MIN_IMU_CORR:
            self.why = f"IMU khong khop goc lai (corr={corr:.2f})"
            return

        yaw = sg * gv[:, ax]
        self.bias = float(np.median(yaw[moving] - yaw_ref[moving]))
        self.yaw = yaw - self.bias                      # tốc độ quay đã trừ độ lệch nền

        dt = np.diff(self.t) / 1e9
        good = (dt > 0) & (dt < 0.1)
        dt = np.where(good, dt, 0.0)
        self.psi = np.concatenate([[0.0], np.cumsum(self.yaw[:-1] * dt)])
        self.X = np.concatenate([[0.0], np.cumsum(self.v[:-1] * np.cos(self.psi[:-1]) * dt)])
        self.Y = np.concatenate([[0.0], np.cumsum(self.v[:-1] * np.sin(self.psi[:-1]) * dt)])
        self.gap = np.concatenate([[0], np.cumsum(~good)])   # đánh dấu chỗ đứt dữ liệu
        self.ok = True
        self.why = f"ok (IMU truc {ax}, dau {sg:+d}, corr={corr:.3f})"

    def path_from(self, t0, span_s=7.0):
        """Quỹ đạo tương đối từ thời điểm t0, trong hệ quy chiếu của xe lúc đó.
        Trả (x tiến, y ngang theo device frame: y dương = PHẢI)."""
        i0 = int(np.searchsorted(self.t, t0))
        i1 = int(np.searchsorted(self.t, t0 + int(span_s * 1e9)))
        if i1 >= len(self.t) or i1 - i0 < 5 or self.gap[i1] != self.gap[i0]:
            return None, None, None
        dX, dY = self.X[i0:i1] - self.X[i0], self.Y[i0:i1] - self.Y[i0]
        c, s = math.cos(-self.psi[i0]), math.sin(-self.psi[i0])
        x = c * dX - s * dY
        y = s * dX + c * dY
        # Dấu của y đã khớp sẵn với device frame của modelV2 (y dương = PHẢI), vì
        # psi được tích phân từ con quay đã căn dấu theo controlsState.curvature.
        # KHÔNG đảo dấu ở đây — selftest.py test B1/C2 chứng minh điều đó.
        return x, y, self.t[i0:i1]

    def lateral_at_time(self, t0, H):
        x, y, tt = self.path_from(t0, H + 0.5)
        if x is None or (tt[-1] - t0) / 1e9 < H - 1e-3:
            return None
        return float(np.interp(t0 + H * 1e9, tt, y))

    def lateral_at_dist(self, t0, D, span_s=12.0):
        x, y, _ = self.path_from(t0, span_s)
        if x is None or x[-1] < D:
            return None
        # np.interp doi truc x tang dan; luc xe bo cham x co the chung chinh
        return float(np.interp(D, np.maximum.accumulate(x), y))


# ═══════════════════════════════════════════════════════════════════════════
# 2. ĐỌC 1 SEGMENT
# ═══════════════════════════════════════════════════════════════════════════
def model_curvature(m, x_lo=5.0, x_hi=25.0):
    """Độ cong quỹ đạo model dự đoán, khớp parabol trong khoảng 5–25m."""
    x, y = np.array(m.position.x), np.array(m.position.y)
    s = (x >= x_lo) & (x <= x_hi)
    if s.sum() < 4:
        return np.nan
    a = np.polyfit(x[s], y[s], 2)[0]
    return float(2 * a)


def read_segment(path, route, seg):
    lg = Log(path)
    tr = Truth(lg)

    cs_ev = lg.get("carState")
    cc_ev = lg.get("carControl")
    mv_ev = lg.get("modelV2")
    if not cs_ev or not cc_ev or not mv_ev:
        return None, dict(route=route, segment=seg, ok=False, why="thieu ban tin co ban")

    cs_t = np.array([e.logMonoTime for e in cs_ev])
    cc_t = np.array([e.logMonoTime for e in cc_ev])
    v_all = np.array([e.carState.vEgo for e in cs_ev])
    lat_all = np.array([bool(e.carControl.latActive) for e in cc_ev])

    # ── kiểm kê (tầng 0) ──────────────────────────────────────────────
    dtc = np.diff(cs_t) / 1e9
    dur = float(np.sum(dtc[(dtc > 0) & (dtc < 1)]))
    km = float(np.sum(v_all[:-1] * np.clip(dtc, 0, 1)) / 1000)
    lat_i = np.interp(cs_t, cc_t, lat_all.astype(float)) > 0.5
    km_eng = float(np.sum((v_all[:-1] * np.clip(dtc, 0, 1))[lat_i[:-1]]) / 1000)
    # số lần nhả lái thật (trên tín hiệu gốc, KHÔNG lọc tốc độ)
    n_diseng = int(np.sum((lat_all[:-1]) & (~lat_all[1:])))
    P = np.array([list(e.modelV2.laneLineProbs) for e in mv_ev])
    E = np.array([list(e.modelV2.roadEdgeStds) for e in mv_ev])
    expo = lg.brightness()
    inv = dict(
        route=route, segment=seg, ok=tr.ok, why=tr.why,
        n_frames=len(mv_ev), duration_s=round(dur, 1), km=round(km, 3), km_engaged=round(km_eng, 3),
        engaged_pct=round(100 * lat_all.mean(), 1),
        v_med_kmh=round(float(np.median(v_all)) * 3.6, 1), v_p90_kmh=round(float(np.percentile(v_all, 90)) * 3.6, 1),
        n_disengage=n_diseng,
        lane_both_p30=round(100 * float(np.mean((P[:, 1] > .3) & (P[:, 2] > .3))), 1),
        lane_both_p50=round(100 * float(np.mean((P[:, 1] > .5) & (P[:, 2] > .5))), 1),
        lane_left_p50=round(100 * float(np.mean(P[:, 1] > .5)), 1),
        lane_right_p50=round(100 * float(np.mean(P[:, 2] > .5)), 1),
        edge_ok_pct=round(100 * float(np.mean((E[:, 0] < .5) | (E[:, 1] < .5))), 1),
        exposure_med=round(float(np.median(expo)), 4) if expo is not None and len(expo) else np.nan,
        imu_corr=round(getattr(tr, "imu_corr", np.nan), 3) if hasattr(tr, "imu_corr") else np.nan,
    )
    if not tr.ok:
        return None, inv

    # ── mức frame: dự đoán quỹ đạo (tầng 2) ──────────────────────────
    st, v = lg.series("carState", "vEgo")
    kt, kk = lg.series("controlsState", "curvature")
    dct = lg.times("controlsState")
    dck = np.array([e.controlsState.desiredCurvature for e in lg.get("controlsState")])
    bt = lg.times("narrowRoadCameraState")
    bv = expo

    rows, prev_k, prev_t = [], np.nan, None
    for e in mv_ev:
        t0, m = e.logMonoTime, e.modelV2
        v0 = float(np.interp(t0, st, v))
        if v0 < MIN_SPEED:
            continue
        eng = bool(lat_all[max(int(np.searchsorted(cc_t, t0, side="right")) - 1, 0)])
        k_now = float(np.interp(t0, kt, kk))
        km_model = model_curvature(m)
        py, ps = np.array(m.position.y), np.array(m.position.yStd)
        px = np.array(m.position.x)
        probs = list(m.laneLineProbs)

        r = dict(route=route, segment=seg, frame_id=m.frameId, t=t0,
                 v_ego=v0, engaged=eng, curvature=k_now,
                 lane_prob_inner=max(probs[1], probs[2]), lane_prob_both=min(probs[1], probs[2]),
                 exposure=float(np.interp(t0, bt, bv)) if bv is not None and len(bt) else np.nan,
                 model_curv=km_model,
                 curv_jitter=abs(km_model - prev_k) if (prev_t is not None and t0 - prev_t < 1e8) else np.nan)
        prev_k, prev_t = km_model, t0

        for h, j in zip(HORIZONS_S, H_IDX):
            yt = tr.lateral_at_time(t0, T_IDXS[j])
            r[f"ystd_{h:.0f}s"] = float(ps[j])
            r[f"err_{h:.0f}s"] = abs(float(py[j]) - yt) if yt is not None else np.nan
            if h == 2.0:      # giữ giá trị CÓ DẤU ở mốc 2s để kiểm tra dấu về sau
                r["y_model_2s"] = float(py[j])
                r["y_thuc_2s"] = yt if yt is not None else np.nan
        for D in HORIZONS_M:
            yt = tr.lateral_at_dist(t0, D)
            if yt is None or px[-1] < D:
                r[f"err_{D:.0f}m"] = np.nan
            else:
                r[f"err_{D:.0f}m"] = abs(float(np.interp(D, px, py)) - yt)
        rows.append(r)

    pred = pd.DataFrame(rows)

    # ── mức cửa sổ: giữ làn (tầng 3) ─────────────────────────────────
    acc = lg.get("accelerometer")
    at = np.array([e.logMonoTime for e in acc]) if acc else np.array([])
    ay = np.array([e.accelerometer.acceleration.v[1] for e in acc]) if acc else np.array([])
    sa_t, sa = lg.series("carState", "steeringAngleDeg")
    lat_t_i = np.interp(tr.t, cc_t, lat_all.astype(float)) > 0.5

    wins = []
    t_start = tr.t[0]
    while t_start + int(WIN_S * 1e9) < tr.t[-1]:
        i0 = int(np.searchsorted(tr.t, t_start))
        i1 = int(np.searchsorted(tr.t, t_start + int(WIN_S * 1e9)))
        t_next = t_start + int(WIN_STEP_S * 1e9)
        if i1 - i0 < 50 or tr.gap[i1] != tr.gap[i0]:
            t_start = t_next
            continue
        sl = slice(i0, i1)
        if tr.v[sl].min() < WIN_MIN_SPEED:
            t_start = t_next
            continue
        eng_frac = float(lat_t_i[sl].mean())
        if eng_frac not in (0.0, 1.0):        # chỉ lấy cửa sổ thuần: hoặc xe lái, hoặc người lái
            t_start = t_next
            continue

        x, y, _ = tr.path_from(tr.t[i0], WIN_S + 0.2)
        if x is None or x[-1] < 8:
            t_start = t_next
            continue
        # Đánh võng = độ lệch so với đường cong trơn khớp tốt nhất, tức đã loại
        # độ cong của chính con đường. Bậc 3: đủ để bám một cung tròn (sai số còn
        # ~1cm, nhỏ so với mức đánh võng thật ~10cm) nhưng chưa đủ mềm để nuốt mất
        # dao động thật. Xem selftest.py test A1–A3.
        coef = np.polyfit(x, y, 3)
        dev = y - np.polyval(coef, x)
        k_win = tr.k[sl]
        ang = np.interp(tr.t[sl], sa_t, sa)
        dang = np.diff(ang)
        reversals = int(np.sum(np.diff(np.sign(dang[np.abs(dang) > 0.05])) != 0))
        if len(at):
            am = (at >= tr.t[i0]) & (at < tr.t[i1])
            if am.sum() > 20:
                # làm mượt 0.2s để bỏ rung do mặt đường, giữ lại chuyển động thật của xe
                w = max(int(0.2 * am.sum() / WIN_S), 3)
                sm = np.convolve(ay[am], np.ones(w) / w, mode="valid")
                tt_ = at[am][w - 1:] / 1e9
                lat_acc = float(np.std(sm))
                jerk = float(np.std(np.diff(sm) / np.diff(tt_))) if len(sm) > 2 else np.nan
            else:
                lat_acc = jerk = np.nan
        else:
            lat_acc = jerk = np.nan
        dk = np.interp(tr.t[sl], dct, dck)
        pm = (pred.t >= tr.t[i0]) & (pred.t < tr.t[i1]) if len(pred) else np.array([], bool)

        wins.append(dict(
            route=route, segment=seg, t=int(tr.t[i0]), engaged=bool(eng_frac == 1.0),
            v_med_kmh=round(float(np.median(tr.v[sl])) * 3.6, 1),
            curv_abs=float(np.median(np.abs(k_win))),
            straight=bool(np.median(np.abs(k_win)) < STRAIGHT_K),
            weave_amp_m=round(float(dev.max() - dev.min()), 4),
            weave_std_m=round(float(np.std(dev)), 4),
            steer_reversals_min=round(reversals / WIN_S * 60, 1),
            lat_accel_std=round(lat_acc, 4) if lat_acc == lat_acc else np.nan,
            lat_jerk_std=round(jerk, 3) if jerk == jerk else np.nan,
            track_err_curv=round(float(np.median(np.abs(dk - k_win))), 6),
            lane_prob_inner=round(float(pred.lane_prob_inner[pm].median()), 3) if pm.any() else np.nan,
            err_2s_med=round(float(pred.err_2s[pm].median()), 4) if pm.any() else np.nan,
            exposure=round(float(pred.exposure[pm].median()), 4) if pm.any() else np.nan,
        ))
        t_start = t_next
    keep = pd.DataFrame(wins)

    # ── sự kiện người can thiệp (tầng 4) ─────────────────────────────
    ev_rows = []
    idx = np.where(lat_all[:-1] & ~lat_all[1:])[0]
    press = np.array([bool(e.carState.steeringPressed) for e in cs_ev])
    pidx = np.where((~press[:-1]) & press[1:])[0]
    for kind, tlist in (("nha_lai", cc_t[idx]), ("day_vo_lang", cs_t[pidx])):
        for te in tlist:
            if kind == "day_vo_lang":
                j = max(int(np.searchsorted(cc_t, te, side="right")) - 1, 0)
                if not lat_all[j]:
                    continue                       # chỉ tính khi Openpilot đang lái
            w0 = te - int(PRECURSOR_S * 1e9)
            pm = ((pred.t >= w0) & (pred.t <= te)).values if len(pred) else np.array([], bool)
            wm = ((keep.t >= w0 - int(WIN_S * 1e9)) & (keep.t <= te)).values if len(keep) else np.array([], bool)
            has = pm.any()
            # luôn ghi sự kiện (để đếm đúng), kể cả khi không có frame nào trong 3s trước
            # (thường là xe đang dừng/bò chậm — đó cũng là một thông tin)
            ev_rows.append(dict(
                route=route, segment=seg, kind=kind, t=int(te),
                v_kmh=round(float(pred.v_ego[pm].median()) * 3.6, 1) if has else np.nan,
                curv_abs=round(float(pred.curvature[pm].abs().median()), 5) if has else np.nan,
                lane_prob_inner=round(float(pred.lane_prob_inner[pm].median()), 3) if has else np.nan,
                ystd_2s=round(float(pred.ystd_2s[pm].median()), 3) if has else np.nan,
                err_2s=round(float(pred.err_2s[pm].median()), 3) if has else np.nan,
                curv_jitter=round(float(pred.curv_jitter[pm].median()), 5) if has else np.nan,
                weave_amp_m=round(float(keep.weave_amp_m[wm].max()), 3) if wm.any() else np.nan,
                exposure=round(float(pred.exposure[pm].median()), 4) if has else np.nan,
                co_du_lieu_truoc=bool(has),
            ))
    ev = pd.DataFrame(ev_rows)
    return (pred, keep, ev), inv


# ═══════════════════════════════════════════════════════════════════════════
# 3. TỔNG HỢP
# ═══════════════════════════════════════════════════════════════════════════
def spearman(a, b):
    a, b = pd.Series(a), pd.Series(b)
    ok = a.notna() & b.notna()
    if ok.sum() < 30:
        return np.nan
    return float(np.corrcoef(a[ok].rank(), b[ok].rank())[0, 1])


def med(s):
    s = pd.Series(s).dropna()
    return float(s.median()) if len(s) else np.nan


def report_prediction(pred):
    print("\n" + "=" * 78)
    print("TANG 2 — LANE PREDICTION: model doan quy dao co dung khong?")
    print("=" * 78)
    print("Dap an = quy dao THUC TE da di (banh xe + IMU), doc lap voi camera/model.\n")
    for lbl, d in (("NGUOI lai (phep do sach)", pred[~pred.engaged]),
                   ("OPENPILOT lai (*)", pred[pred.engaged])):
        if len(d) < 50:
            print(f"-- {lbl}: khong du mau ({len(d)})"); continue
        print(f"-- {lbl}  n={len(d)}")
        for h in HORIZONS_S:
            e, s = d[f"err_{h:.0f}s"], d[f"ystd_{h:.0f}s"]
            ok = e.notna() & s.notna() & (s > 0)
            if ok.sum() < 30:
                continue
            e, s = e[ok], s[ok]
            print(f"   {h:.0f}s: sai so median={e.median():.3f}m  P90={e.quantile(.9):.3f}m  "
                  f">0.5m: {100*(e>0.5).mean():4.1f}%  |  yStd={s.median():.3f}m  "
                  f"ty le={np.median(e/s):.2f}  xep hang={spearman(s,e):+.2f}")
        row = "   theo quang duong: " + "  ".join(
            f"{D:.0f}m={med(d[f'err_{D:.0f}m']):.3f}m" for D in HORIZONS_M)
        print(row)
    print("\n(*) Khi Openpilot lai, xe di theo chinh du doan cua no nen sai so nho hon thuc chat;")
    print("    khong dung con so nay de danh gia do chinh xac du doan.")
    print(f"\nDo rung quy dao (curv_jitter) median: nguoi lai={med(pred[~pred.engaged].curv_jitter):.5f}"
          f"  openpilot lai={med(pred[pred.engaged].curv_jitter):.5f} 1/m")


def report_keeping(keep):
    print("\n" + "=" * 78)
    print("TANG 3 — LANE KEEPING: xe giu lan co on khong?")
    print("=" * 78)
    if len(keep) < 20:
        print("Khong du cua so de danh gia."); return
    print("Do bang cam bien vat ly, khong can vach ke. Cua so 5 giay.\n")
    print(f"{'nhom':28s} {'n':>5} {'vong(m)':>9} {'doi chieu/ph':>13} {'accel ngang':>12} {'giat ngang':>11} {'bam lenh':>10}")
    for straight in (True, False):
        for eng in (True, False):
            d = keep[(keep.straight == straight) & (keep.engaged == eng)]
            if len(d) < 10:
                continue
            lbl = f"{'Thang' if straight else 'Cua  '} | {'Openpilot' if eng else 'Nguoi   '}"
            print(f"{lbl:28s} {len(d):>5} {med(d.weave_amp_m):>9.3f} {med(d.steer_reversals_min):>13.0f} "
                  f"{med(d.lat_accel_std):>12.3f} {med(d.lat_jerk_std):>11.2f} "
                  f"{med(d.track_err_curv):>10.5f}")
    print("\n  vong(m)      = bien do danh vong trong 5s (da loai do cong cua duong)")
    print("  bam lenh     = |desiredCurvature - do cong thuc| (chi co nghia khi Openpilot lai)")
    e = keep[keep.engaged]
    if len(e) > 30:
        print(f"\nTuong quan (khi Openpilot lai): danh vong ~ toc do {spearman(e.v_med_kmh, e.weave_amp_m):+.2f} | "
              f"~ do cong {spearman(e.curv_abs, e.weave_amp_m):+.2f} | ~ nhin thay vach {spearman(e.lane_prob_inner, e.weave_amp_m):+.2f}")


def report_events(ev, pred, inv):
    print("\n" + "=" * 78)
    print("TANG 4 — SU KIEN NGUOI CAN THIEP va dau hieu 3 giay truoc do")
    print("=" * 78)
    km = inv.km_engaged.sum()
    if not len(ev):
        print("Khong co su kien nao."); return
    print(f"Tong: {len(ev)} lan can thiep / {km:.1f} km lai tu dong = {len(ev)/max(km,1e-9):.1f} lan/km")
    print(f"(day vo lang va nha lai cach nhau vai mili-giay la MOT lan, da gop)")
    print(f"Doi chieu: {int(inv.n_disengage.sum())} lan nha lai dem tho tren tin hieu goc.")

    d = ev[ev.co_du_lieu_truoc == True]
    base = pred[pred.engaged].copy()
    if len(d) < 10 or len(base) < 500:
        print("Khong du mau de phan tich dau hieu bao truoc."); return
    base["v_kmh"] = base.v_ego * 3.6

    # So voi nen CUNG DAI TOC DO — vi can thiep hay xay ra luc chay cham, neu so
    # voi nen chung thi toc do se danh lua, lam moi dau hieu trong manh hon thuc te.
    rows = []
    for _, r in d.iterrows():
        b = base[(base.v_kmh - r.v_kmh).abs() < 3]
        if len(b) < 200:
            continue
        rows.append(dict(
            mat_vach=r.lane_prob_inner < LANE_PROB_OK, nen_mat_vach=(b.lane_prob_inner < LANE_PROB_OK).mean(),
            ystd_cao=r.ystd_2s > 0.4, nen_ystd_cao=(b.ystd_2s > 0.4).mean(),
            vao_cua=abs(r.curv_abs) > STRAIGHT_K, nen_vao_cua=(b.curvature.abs() > STRAIGHT_K).mean()))
    m = pd.DataFrame(rows)
    if len(m) < 10:
        print("Khong du su kien doi chieu duoc theo toc do."); return
    print(f"\nDau hieu 3 giay truoc, so voi nen CUNG DAI TOC DO (+-3 km/h), n={len(m)}:")
    print(f"  {'dau hieu':22s} {'truoc can thiep':>16s} {'nen cung toc do':>16s} {'ty le':>8s}")
    for a, b_, lbl in (("ystd_cao", "nen_ystd_cao", "yStd_2s > 0.4m"),
                       ("mat_vach", "nen_mat_vach", f"mat vach (<{LANE_PROB_OK})"),
                       ("vao_cua", "nen_vao_cua", "dang vao cua")):
        o, n = m[a].mean(), m[b_].mean()
        print(f"  {lbl:22s} {100*o:>15.1f}% {100*n:>15.1f}% {o/max(n,1e-9):>7.2f}x")


def report_envelope(pred, keep, ev, inv, outdir):
    print("\n" + "=" * 78)
    print("TANG 5 — BAN DO BIEN VAN HANH (chi tinh luc Openpilot lai)")
    print("=" * 78)
    p = pred[pred.engaged].copy()
    k = keep[keep.engaged].copy()
    if not len(p):
        print("Khong co du lieu."); return
    p["vbin"] = pd.cut(p.v_ego * 3.6, [0, 30, 50, 200], labels=["<30", "30-50", ">50"])
    p["shape"] = np.where(p.curvature.abs() < STRAIGHT_K, "thang", "cua")
    p["lane"] = np.where(p.lane_prob_inner > LANE_PROB_OK, "co vach", "mat vach")
    k["vbin"] = pd.cut(k.v_med_kmh, [0, 30, 50, 200], labels=["<30", "30-50", ">50"])
    k["shape"] = np.where(k.straight, "thang", "cua")
    k["lane"] = np.where(k.lane_prob_inner > LANE_PROB_OK, "co vach", "mat vach")

    g = p.groupby(["vbin", "shape", "lane"], observed=True).agg(
        n_frame=("err_2s", "size"), err_2s=("err_2s", "median"),
        err_2s_p90=("err_2s", lambda s: s.quantile(.9)), ystd_2s=("ystd_2s", "median")).reset_index()
    gk = k.groupby(["vbin", "shape", "lane"], observed=True).agg(
        n_win=("weave_amp_m", "size"), weave=("weave_amp_m", "median"),
        track=("track_err_curv", "median")).reset_index()
    env = g.merge(gk, on=["vbin", "shape", "lane"], how="outer")
    env["du_ket_luan"] = np.where(env.n_frame.fillna(0) >= 100, "co", "MAU QUA IT")
    env.to_csv(os.path.join(outdir, "05_envelope.csv"), index=False)
    print(env.round(4).to_string(index=False))

    # đoạn tệ nhất để xem lại bằng video
    clips = []
    if len(p):
        w = p.nlargest(15, "err_2s")[["route", "segment", "frame_id", "v_ego", "err_2s", "ystd_2s", "lane_prob_inner"]]
        w = w.assign(ly_do="sai so du doan lon")
        clips.append(w)
    if len(k):
        w2 = k.nlargest(15, "weave_amp_m")[["route", "segment", "t", "v_med_kmh", "weave_amp_m", "lane_prob_inner"]]
        w2 = w2.assign(ly_do="danh vong manh")
        clips.append(w2)
    if clips:
        cl = pd.concat(clips, ignore_index=True)
        cl.to_csv(os.path.join(outdir, "05_clips.csv"), index=False)
        print(f"\nDa luu {len(cl)} doan dang xem lai -> 05_clips.csv")


# ═══════════════════════════════════════════════════════════════════════════
# 4. CHẠY
# ═══════════════════════════════════════════════════════════════════════════
def find_segments(paths):
    """Nhận: thư mục route (chứa segment_*), hoặc thư mục chứa nhiều route."""
    out = []
    for p in paths:
        p = os.path.abspath(os.path.expanduser(p))
        if not os.path.isdir(p):
            print(f"[bo qua] khong phai thu muc: {p}"); continue
        subs = sorted(os.listdir(p))
        has_seg = any(s.startswith("segment_") and os.path.isdir(os.path.join(p, s)) for s in subs)
        routes = [(os.path.basename(p), p)] if has_seg else [
            (s, os.path.join(p, s)) for s in subs if os.path.isdir(os.path.join(p, s))]
        for rname, rpath in routes:
            segs = [s for s in sorted(os.listdir(rpath))
                    if s.startswith("segment_") and os.path.isdir(os.path.join(rpath, s))]
            segs.sort(key=lambda s: int(s.split("_")[-1]) if s.split("_")[-1].isdigit() else 0)
            for s in segs:
                f = next((os.path.join(rpath, s, n) for n in ("rlog.zst", "rlog.bz2", "rlog")
                          if os.path.isfile(os.path.join(rpath, s, n))), None)
                if f:
                    out.append((rname, s, f))
    return out


def main(paths, max_seg=None, outdir="."):
    segs = find_segments(paths)
    if max_seg:
        segs = segs[:max_seg]
    if not segs:
        print("Khong tim thay segment nao. Kiem tra duong dan."); return
    routes = sorted({r for r, _, _ in segs})
    print(f"Tim thay {len(segs)} segment tu {len(routes)} route: {', '.join(routes)}\n")

    preds, keeps, evs, invs = [], [], [], []
    for n, (route, seg, f) in enumerate(segs, 1):
        try:
            res, inv = read_segment(f, route, seg)
        except Exception as ex:
            print(f"  [{n}/{len(segs)}] {route}/{seg}: LOI {type(ex).__name__}: {ex}")
            invs.append(dict(route=route, segment=seg, ok=False, why=f"loi: {ex}"))
            continue
        invs.append(inv)
        if res is None:
            print(f"  [{n}/{len(segs)}] {route}/{seg}: bo qua — {inv.get('why')}")
            continue
        p, k, e = res
        preds.append(p); keeps.append(k); evs.append(e)
        print(f"  [{n}/{len(segs)}] {route}/{seg}: {len(p)} frame, {len(k)} cua so, {len(e)} su kien")

    inv = pd.DataFrame(invs)
    inv.to_csv(os.path.join(outdir, "00_inventory.csv"), index=False)
    pred = pd.concat(preds, ignore_index=True) if preds else pd.DataFrame()
    keep = pd.concat(keeps, ignore_index=True) if keeps else pd.DataFrame()
    ev = pd.concat(evs, ignore_index=True) if evs else pd.DataFrame()
    # Đẩy vô-lăng rồi nhả lái cách nhau vài mili-giây là MỘT lần can thiệp, không
    # phải hai. Gộp các sự kiện trong cùng segment cách nhau dưới 1 giây.
    if len(ev):
        ev = ev.sort_values(["route", "segment", "t"]).reset_index(drop=True)
        gap = ev.groupby(["route", "segment"]).t.diff()
        ev["su_kien_moi"] = gap.isna() | (gap > 1e9)
        n_tho = len(ev)
        ev = ev[ev.su_kien_moi].drop(columns="su_kien_moi")
        print(f"\nGop su kien: {n_tho} dong tho -> {len(ev)} lan can thiep that su")
    for df, name in ((pred, "02_prediction.csv"), (keep, "03_keeping.csv"), (ev, "04_events.csv")):
        if len(df):
            df.to_csv(os.path.join(outdir, name), index=False)

    # ── tầng 0: kiểm kê ──────────────────────────────────────────────
    good = inv[inv.ok == True] if "ok" in inv else inv
    print("\n" + "=" * 78)
    print("TANG 0 — KIEM KE DU LIEU")
    print("=" * 78)
    print(f"Segment dung duoc: {len(good)}/{len(inv)}")
    if len(good):
        print(f"Thoi gian: {good.duration_s.sum()/60:.0f} phut | quang duong {good.km.sum():.1f} km "
              f"| Openpilot lai {good.km_engaged.sum():.1f} km ({good.engaged_pct.mean():.0f}% thoi gian)")
        print(f"Toc do median {good.v_med_kmh.median():.0f} km/h, P90 {good.v_p90_kmh.max():.0f} km/h")
        print(f"So lan nha lai: {int(good.n_disengage.sum())} "
              f"({good.n_disengage.sum()/max(good.km_engaged.sum(),1e-9):.1f} lan/km)")
        print(f"Nhin thay CA HAI vach (>0.3): {good.lane_both_p30.mean():.1f}% frame | "
              f"vach trai {good.lane_left_p50.mean():.1f}% | vach phai {good.lane_right_p50.mean():.1f}%")
        ex = good.exposure_med.dropna()
        if len(ex):
            print(f"Do sang (exposure): median {ex.median():.3f}, max {ex.max():.3f} "
                  f"({'co doan toi/dem' if ex.max() > 5*ex.median() else 'gan nhu chi ban ngay'})")
    bad = inv[inv.ok != True] if "ok" in inv else pd.DataFrame()
    if len(bad):
        print(f"\nSegment bo qua ({len(bad)}):")
        for _, r in bad.iterrows():
            print(f"   {r.get('route')}/{r.get('segment')}: {r.get('why')}")

    # ── chốt chặn: dấu của đáp án phải khớp với model ────────────────
    if len(pred) > 200:
        d = pred[["y_model_2s", "y_thuc_2s"]].dropna()
        r = float(np.corrcoef(d.y_model_2s, d.y_thuc_2s)[0, 1]) if len(d) > 100 else np.nan
        if np.isfinite(r) and r < 0.5:
            print("\n" + "!" * 78)
            print(f"DUNG LAI: dap an va model khong khop dau (tuong quan = {r:+.3f}).")
            print("Moi con so sai so ben duoi deu VO NGHIA. Chay selftest.py de tim nguyen nhan.")
            print("!" * 78)
        else:
            print(f"\n[kiem tra dau] tuong quan model vs thuc te o moc 2s = {r:+.3f}  (can > 0.5)")

    if len(pred):
        report_prediction(pred)
    if len(keep):
        report_keeping(keep)
    if len(ev) and len(good):
        report_events(ev, pred, good)
    if len(pred) and len(good):
        report_envelope(pred, keep, ev, good, outdir)
    print(f"\nDa ghi cac file CSV vao: {os.path.abspath(outdir)}")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    mx = None
    for a in sys.argv[1:]:
        if a.startswith("--max-seg"):
            mx = int(a.split("=")[1]) if "=" in a else int(sys.argv[sys.argv.index(a) + 1])
    if not args:
        print(__doc__)
        sys.exit(1)
    main(args, max_seg=mx)
