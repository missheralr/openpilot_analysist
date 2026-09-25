"""
verify_doc.py — Kiem chung lai TUNG con so va TUNG trich dan trong tai lieu
"openpilot_vf6_phan_tich_he_thong.docx".

Muc dich: khong ai phai tin loi nguoi viet tai lieu. Chay script nay, no se
tu doc lai log va tu mo lai ma nguon, roi so voi con so ghi trong tai lieu.

    python verify_doc.py <rlog.zst> <repo_fork> [repo_goc_cua_comma]

Vi du:
    python verify_doc.py ~/openpilot/data/segment_22/rlog.zst ~/qmpilot

Moi dong in ra co dang:
    [KHOP]    ten muc        tai lieu ghi X   do lai duoc Y
    [LECH]    ...
    [KHONG KIEM TRA DUOC] ... kem ly do

Cuoi cung in tong ket. Bat ky dong LECH nao cung co nghia tai lieu sai o do.
"""

import os
import sys
import math
import numpy as np

OK, BAD, SKIP = [], [], []


def cmp(name, doc_val, got, tol=0.02, unit=""):
    """So sanh con so trong tai lieu voi con so do lai. tol = sai lech tuong doi cho phep."""
    if got is None:
        SKIP.append(name); print(f"  [KHONG DO DUOC] {name}"); return
    ok = abs(got - doc_val) <= max(tol * abs(doc_val), 1e-9)
    (OK if ok else BAD).append(name)
    print(f"  [{'KHOP' if ok else 'LECH'}] {name:46s} tai lieu {doc_val:>10.4g}{unit}   do duoc {got:>10.4g}{unit}")


def cmp_str(name, doc_val, got):
    ok = str(got) == str(doc_val)
    (OK if ok else BAD).append(name)
    print(f"  [{'KHOP' if ok else 'LECH'}] {name:46s} tai lieu {doc_val!r}   doc duoc {got!r}")


def lead_present(R):
    """Ban goc goi truong nay la 'present', ban fork goi la 'status' — cung mot du lieu."""
    return bool(getattr(R, "present", None) if hasattr(R, "present") else R.status)


def cite(repo, relpath, line_no, must_contain):
    """Mo dung dong ma nguon ma tai lieu trich dan, kiem tra noi dung co khop khong."""
    f = os.path.join(repo, relpath)
    if not os.path.isfile(f):                       # mot so ban co them thu muc openpilot/
        alt = os.path.join(repo, "openpilot", relpath)
        if os.path.isfile(alt):
            f = alt
    name = f"{relpath}:{line_no}"
    if not os.path.isfile(f):
        SKIP.append(name); print(f"  [KHONG CO TEP] {name}"); return
    lines = open(f, encoding="utf-8", errors="replace").read().splitlines()
    win = " ".join(lines[max(line_no - 3, 0): line_no + 3])
    ok = must_contain in win
    (OK if ok else BAD).append(name)
    print(f"  [{'KHOP' if ok else 'LECH'}] {name:56s} tim '{must_contain[:34]}'")


# ══════════════════════════════════════════════════════════════════════════
def main(rlog, repo, upstream=None):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import oplog
    # BAT BUOC dung schema cua chinh ban fork, neu khong ten enum se sai
    # Uu tien bien moi truong — de dung duoc ca khi schema nam rieng mot cho,
    # khong nhat thiet phai co ca repo ma nguon.
    oplog.CEREAL = os.environ.get("CEREAL_DIR") or os.path.join(repo, "cereal")
    oplog.OPENDBC = os.environ.get("OPENDBC_DIR") or os.path.join(repo, "opendbc_repo/opendbc/car")
    lg = oplog.Log(rlog)

    cs = lg.get("carState"); mv = lg.get("modelV2"); rs = lg.get("radarState")
    ctl = lg.get("controlsState"); lp = lg.get("longitudinalPlan")
    cp = lg.first("carParams")
    t = np.array([e.logMonoTime for e in cs]); v = np.array([e.carState.vEgo for e in cs])
    cc_t = np.array([e.logMonoTime for e in lg.get("carControl")])
    latA = np.array([bool(e.carControl.latActive) for e in lg.get("carControl")])

    print("\n" + "=" * 96)
    print("A. NHUNG CON SO DO TU LOG")
    print("=" * 96)

    me = np.array([e.modelV2.modelExecutionTime for e in mv]) * 1000
    cmp("Thoi gian chay mang (trung vi, ms)", 25.8, float(np.median(me)), unit=" ms")

    T = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
    ys = np.array([list(e.modelV2.position.yStd) for e in mv])
    for lbl, i, doc in (("yStd t=0.98s", 10, 0.052), ("yStd t=1.91s", 14, 0.131),
                        ("yStd t=3.16s", 18, 0.215), ("yStd t=10.0s", 32, 1.129)):
        cmp(lbl, doc, float(np.median(ys[:, i])), tol=0.03, unit=" m")
    cmp("Moc thoi gian chi so 16 (phai la 2.5s)", 2.5, float(T[16]), unit=" s")

    d = lg.first("liveDelay")
    cmp("liveDelay.lateralDelay", 0.326, float(d.lateralDelay) if d else None, unit=" s")
    cmp("steerActuatorDelay", 0.050, float(cp.steerActuatorDelay), unit=" s")
    cmp("lat_action_t = delay + 0.075", 0.401, (float(d.lateralDelay) + 0.075) if d else None, unit=" s")
    cmp("long_action_t", 0.875, float(cp.longitudinalActuatorDelay) + 0.3 + 0.075, unit=" s")

    n_ang = sum(1 for e in ctl if e.controlsState.lateralControlState.which() == "angleState")
    cmp("So ban tin controlsState", 5988, len(ctl))
    cmp("Trong do la angleState", 5988, n_ang)
    cmp_str("carFingerprint", "VINFAST_VF6", cp.carFingerprint)
    cmp_str("safetyModel", "vinfastVf6", str(cp.safetyConfigs[0].safetyModel))

    ten_ts = lg.pick("liveParameters", "vehicleParameters")
    P = lg.first(ten_ts)
    cmp("steerRatio hoc duoc", 14.72, float(P.steerRatio))
    cmp("stiffnessFactor hoc duoc", 0.98, float(P.stiffnessFactor))
    cmp("angleOffsetDeg hoc duoc", -1.09, float(P.angleOffsetDeg), unit=" do")
    cmp("steerRatio trong cau hinh goc", 15.5, float(cp.steerRatio))

    # goc co ban vs phan PI
    m_, wb = cp.mass, cp.wheelbase
    aF = cp.centerToFront; aR = wb - aF
    cF0, cR0, chi = cp.tireStiffnessFront, cp.tireStiffnessRear, cp.steerRatioRear
    def cfac(u, s):
        cF, cR = s * cF0, s * cR0
        sf = -(m_ * (cF * aF - cR * aR) / (wb ** 2)) / (cF * cR)
        return (1. - chi) / (1. - sf * u ** 2) / wb
    ff, cmdv, act = [], [], []
    for e in ctl:
        st = e.controlsState.lateralControlState
        if st.which() != "angleState" or not st.angleState.active:
            continue
        c_ = lg.at("carState", e.logMonoTime); pp = lg.at(ten_ts, e.logMonoTime)
        u = max(c_.vEgo, 0.1)
        ff.append(-(math.degrees(e.controlsState.desiredCurvature * pp.steerRatio / cfac(u, pp.stiffnessFactor)) + pp.angleOffsetDeg))
        cmdv.append(st.angleState.steeringAngleDesiredDeg); act.append(c_.steeringAngleDeg)
    ff, cmdv, act = np.array(ff), np.array(cmdv), np.array(act)
    cmp("Goc co ban tu do cong (trung vi)", 4.93, float(np.median(np.abs(ff))), tol=0.03, unit=" do")
    cmp("Phan PI cong them (trung vi)", 5.09, float(np.median(np.abs(cmdv - ff))), tol=0.03, unit=" do")
    cmp("He so thuc hien / lenh day du", 0.44, float(np.polyfit(cmdv, act, 1)[0]), tol=0.05)
    cmp("He so thuc hien / goc co ban", 0.78, float(np.polyfit(ff, act, 1)[0]), tol=0.05)

    # thien lech trai
    st_, vv = lg.series("carState", "vEgo")
    bias, kk = [], []
    for e in mv:
        tt = e.logMonoTime
        if not latA[max(int(np.searchsorted(cc_t, tt, side="right")) - 1, 0)]:
            continue
        k = float(e.modelV2.action.desiredCurvature); u = float(np.interp(tt, st_, vv)) * 3.6
        if u >= 60: o = 0.0
        else:
            om = -0.0015 if u < 20 else (-0.001 if u < 40 else -0.0005 * math.exp(-((u - 40) / 20) * 4.0))
            o = om * math.exp(-abs(k) / 0.002)
        bias.append(abs(o)); kk.append(abs(k))
    bias, kk = np.array(bias), np.array(kk)
    cmp("Thien lech trai khi gan nhu di thang", 0.00108, float(np.median(bias[kk < 0.0005])), tol=0.05, unit=" 1/m")
    cmp("Thien lech trai khi dang vao cua", 0.00020, float(np.median(bias[kk >= 0.002])), tol=0.10, unit=" 1/m")

    # gioi han ISO
    cmp("Ban kinh cua nho nhat o 60 km/h", 93.0, 1 / (3.0 / (60 / 3.6) ** 2), tol=0.02, unit=" m")
    cmp("Ban kinh cua nho nhat o 100 km/h", 260.0, 1 / (3.0 / (100 / 3.6) ** 2), tol=0.02, unit=" m")

    # khoang cach xe truoc
    CB, SD, TF = 2.5, 6.0, 1.75
    rows = []
    for e in rs:
        R = e.radarState.leadOne
        if not lead_present(R): continue
        c_ = lg.at("carState", e.logMonoTime); cc = lg.at("carControl", e.logMonoTime)
        if not cc.longActive or c_.vEgo < 2: continue
        rows.append((R.dRel, c_.vEgo ** 2 / (2 * CB) + TF * c_.vEgo + SD))
    a = np.array(rows)
    cmp("So frame doi chieu khoang cach", 487, len(a), tol=0.02)
    cmp("Khoang cach xe truoc thuc te", 28.9, float(np.median(a[:, 0])), tol=0.03, unit=" m")
    cmp("Khoang cach MPC mong muon", 24.4, float(np.median(a[:, 1])), tol=0.03, unit=" m")
    cmp_str("Che do lai (personality)", "relaxed", str(lg.first("selfdriveState").personality))

    sv = np.array([x.longitudinalPlan.solverExecutionTime for x in lp]) * 1000
    cmp("Thoi gian giai MPC (trung vi)", 0.59, float(np.median(sv)), tol=0.20, unit=" ms")
    src = [str(x.longitudinalPlan.longitudinalPlanSource) for x in lp]
    cmp("Ty le nguon cruise", 55.0, 100 * src.count("cruise") / len(src), tol=0.05, unit=" %")
    cmp("Ty le nguon e2e", 36.0, 100 * src.count("e2e") / len(src), tol=0.05, unit=" %")

    # thi giac vs radar
    mvd = {e.logMonoTime: e.modelV2 for e in mv}
    hit = sum(1 for e in rs if e.radarState.mdMonoTime in mvd)
    cmp("mdMonoTime khop modelV2 (%)", 100.0, 100 * hit / len(rs), unit=" %")
    V, R_, S, VO = [], [], [], []
    for e in rs:
        R = e.radarState.leadOne
        if not lead_present(R): continue
        m = mvd.get(e.radarState.mdMonoTime)
        if m is None or not len(m.leadsV3) or m.leadsV3[0].prob < 0.5: continue
        if R.radar and R.radarTrackId != -1:
            V.append(float(m.leadsV3[0].x[0]) - 1.52); R_.append(float(R.dRel)); S.append(float(m.leadsV3[0].xStd[0]))
        else:
            VO.append(float(m.leadsV3[0].x[0]) - float(R.dRel))
    V, R_, S = np.array(V), np.array(R_), np.array(S)
    cmp("Quy uoc RADAR_TO_CAMERA (nhanh thi giac)", 1.52, float(np.median(VO)), unit=" m")
    cmp("Lech thi giac so voi radar", -3.63, float(np.median(V - R_)), tol=0.03, unit=" m")
    bi = np.median(V - R_)
    cmp("Do tan mac quanh muc lech", 0.90, float(np.median(np.abs(V - R_ - bi))), tol=0.05, unit=" m")
    cmp("Ty le sai so tren xStd", 1.72, float(np.median(np.abs(V - R_) / S)), tol=0.05)
    mask = (R_ >= 10) & (R_ < 20)
    cmp("Lech o cu ly 10-20 m", -0.12, float(np.median((V - R_)[mask])), tol=0.40, unit=" m")

    pres = [e.radarState.leadOne for e in rs if lead_present(e.radarState.leadOne)]
    cmp("Ty le frame co radar hau thuan", 38.0, 100 * np.mean([bool(x.radar and x.radarTrackId != -1) for x in pres]), tol=0.05, unit=" %")
    lt = lg.get("liveTracks")
    npt = np.array([len(e.liveTracks.points) for e in lt])
    cmp("So ban tin liveTracks", 1564, len(lt), tol=0.02)
    cmp("Ty le ban tin khong co diem radar", 28.0, 100 * float(np.mean(npt == 0)), tol=0.05, unit=" %")
    pts = [p for e in lt for p in e.liveTracks.points]
    cmp("Tong so diem radar", 1337, len(pts), tol=0.02)
    cmp("Ty le diem co co measured", 0.0, 100 * float(np.mean([p.measured for p in pts])), tol=0, unit=" %")
    cmp("Ty le diem co van toc khac 0", 72.0, 100 * float(np.mean([p.vRel != 0 for p in pts])), tol=0.05, unit=" %")

    from collections import Counter
    evc = Counter(str(x.name) for e in lg.get("onroadEvents") for x in e.onroadEvents)
    for k, n in (("cruiseMismatch", 28), ("pedalPressed", 10), ("steerOverride", 7), ("pcmDisable", 1)):
        cmp(f"So su kien {k}", n, evc.get(k, 0), tol=0)
    ps = lg.get("pandaStates")
    cmp("controlsAllowed (%)", 100.0, 100 * float(np.mean([e.pandaStates[0].controlsAllowed for e in ps])), unit=" %")
    cmp("Ty le thoi gian lai tu dong", 43.0, 100 * float(latA.mean()), tol=0.05, unit=" %")

    print("\n" + "=" * 96)
    print("B. NHUNG TRICH DAN MA NGUON — mo dung dong tai lieu ghi, kiem tra noi dung")
    print("=" * 96)
    for rel, ln, txt in [
        ("selfdrive/modeld/modeld.py", 136, "run_policy"),
        ("selfdrive/modeld/modeld.py", 98, "MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ"),
        ("selfdrive/modeld/modeld.py", 124, "prev_desire > .99"),
        ("selfdrive/modeld/modeld.py", 55, "action'][0,0] / (max(1.0, v_ego))**2"),
        ("selfdrive/modeld/modeld.py", 299, "lat_action_t = lat_delay + frame_delay + action_delay"),
        ("selfdrive/modeld/modeld.py", 35, "LAT_SMOOTH_SECONDS = 0.0"),
        ("selfdrive/modeld/parse_model_outputs.py", 52, "pred_std = safe_exp"),
        ("selfdrive/modeld/parse_model_outputs.py", 113, "parse_mdn('plan', outs, in_N=0"),
        ("selfdrive/modeld/compile_modeld.py", 121, "features_buffer"),
        ("selfdrive/controls/controlsd.py", 192, "model_v2.action.desiredCurvature if CC.latActive"),
        ("selfdrive/controls/controlsd.py", 202, 'self.CP.brand == "vinfast"'),
        ("selfdrive/controls/controlsd.py", 226, "curv_scale = 0.002"),
        ("selfdrive/controls/lib/drive_helpers.py", 29, "MAX_LATERAL_JERK / (v_ego ** 2)"),
        ("selfdrive/controls/lib/drive_helpers.py", 14, "MAX_LATERAL_JERK = 5.0"),
        ("opendbc_repo/opendbc/car/vehicle_model.py", 104, "self.sR * 1.0 / self.curvature_factor(u)"),
        ("selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py", 100, "model.u = vertcat(j_ego)"),
        ("selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py", 40, "A_CHANGE_COST = 200."),
        ("selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py", 90, "t_follow * v_ego + STOP_DISTANCE"),
        ("selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py", 354, "MPC_SOURCES[np.argmin(x_obstacles[0])]"),
        ("selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py", 79, "return 1.75"),
        ("selfdrive/controls/lib/longitudinal_planner.py", 329, "min(output_a_target_e2e, output_a_target_mpc)"),
        ("selfdrive/controls/lib/longitudinal_planner.py", 58, "VF_MILD_DECEL_SCALE = 0.93"),
        ("selfdrive/controls/lib/longitudinal_planner.py", 155, "VF_REDLIGHT_FINGERPRINTS"),
        ("selfdrive/controls/radard.py", 1, "_BYTECODE_SHIM"),   # fork: chi la vo, ma that trong .pyc
        ("selfdrive/selfdrived/state.py", 7, "SOFT_DISABLE_TIME = 3"),
        ("selfdrive/selfdrived/events.py", 281, "#ET.PERMANENT"),
        ("opendbc/safety/modes/vinfast_stub.h", 0, "controls_allowed = true"),
    ]:
        p = rel if os.path.exists(os.path.join(repo, rel)) else os.path.join("opendbc_repo", rel)
        cite(repo, p, ln if ln else 10, txt)

    if upstream:
        print("\n  -- trich dan lay tu BAN GOC comma (fork dong goi radard thanh bytecode) --")
        for ln, txt in ((27, "RADAR_TO_CAMERA = 1.52"), (76, "_LEAD_ACCEL_TAU"), (117, "laplacian_pdf(c.dRel")):
            cite(upstream, "selfdrive/controls/radard.py", ln, txt)
    else:
        print("\n  (bo qua 3 trich dan radard cua ban goc — truyen them duong dan repo goc de kiem tra)")

    print("\n" + "=" * 96)
    print(f"TONG KET: {len(OK)} KHOP | {len(BAD)} LECH | {len(SKIP)} khong kiem tra duoc")
    if BAD:
        print("\nCac muc LECH — tai lieu sai o day:")
        for b in BAD: print("   -", b)
    print("=" * 96)
    print("\nLUU Y: nhung phan KHONG kiem tra duoc bang script nay:")
    print("  1. Firmware panda that su dang chay tren xe — ma dong, khong doc duoc.")
    print("  2. Hang so trong radard va latcontrol_angle cua fork — nam trong .pyc,")
    print("     phai dich nguoc bang python3.12 (xem muc 8.1 cua tai lieu).")
    print("  3. Cac cau dien giai va nhan dinh thiet ke — khong phai su kien do duoc.")
    return 1 if BAD else 0


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    up = os.path.expanduser(sys.argv[3]) if len(sys.argv) > 3 else None
    sys.exit(main(os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]), up))
