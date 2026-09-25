"""
phase1_modeld.py — Sinh ra moi con so cua Phase 1 trong tai lieu (khoi modeld).

    python phase1_modeld.py <rlog.zst> <repo_fork>

In ra dung nhung gi da dua vao muc "Phase 1 — modeld" cua tai lieu:
  1.1  hai mang: vision + policy
  1.2  bo dem dac trung 25 x 512, frame_skip, ky uc 5 giay
  1.3  desire la xung suon len
  1.4  moi dau ra kem do lech chuan; bang yStd theo tam nhin
  1.5  lenh lai la gia toc ngang chia cho v binh phuong
  1.6  do tre hoc truc tuyen va tam nhin cua lenh lai
"""
import os
import sys
import numpy as np


def grep(repo, rel, pattern, n=3):
    """In cac dong ma nguon khop pattern — de doi chieu voi trich dan trong tai lieu."""
    f = os.path.join(repo, rel)
    if not os.path.isfile(f):
        print(f"    (khong thay {rel})"); return
    for i, line in enumerate(open(f, encoding="utf-8", errors="replace"), 1):
        if pattern in line:
            print(f"    {rel}:{i}  {line.rstrip()}")
            n -= 1
            if n <= 0:
                return


def main(rlog, repo):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import oplog
    # Uu tien bien moi truong — de dung duoc ca khi schema nam rieng mot cho,
    # khong nhat thiet phai co ca repo ma nguon.
    oplog.CEREAL = os.environ.get("CEREAL_DIR") or os.path.join(repo, "cereal")
    oplog.OPENDBC = os.environ.get("OPENDBC_DIR") or os.path.join(repo, "opendbc_repo/opendbc/car")
    lg = oplog.Log(rlog)
    mv = lg.get("modelV2")

    print("\n" + "=" * 78)
    print("1.1  HAI MANG: thi giac va chinh sach")
    print("=" * 78)
    grep(repo, "selfdrive/modeld/modeld.py", "run_policy(")

    print("\n" + "=" * 78)
    print("1.2  KY UC CUA MODEL")
    print("=" * 78)
    grep(repo, "selfdrive/modeld/compile_modeld.py", "features_buffer")
    grep(repo, "selfdrive/modeld/modeld.py", "MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ")
    grep(repo, "selfdrive/modeld/constants.py", "MODEL_CONTEXT_FREQ")
    print("    -> frame_skip = 20 / 5 = 4 ; bo dem 25 mau x 4 frame / 20 Hz = 5,0 giay")
    print("    -> anh vao: (1, 12, 128, 256) = 2 khung x 6 kenh, hai khung cach nhau 4 frame = 0,2 s")

    print("\n" + "=" * 78)
    print("1.3  DESIRE LA XUNG, KHONG PHAI TRANG THAI")
    print("=" * 78)
    grep(repo, "selfdrive/modeld/modeld.py", "prev_desire > .99")

    print("\n" + "=" * 78)
    print("1.4  MOI DAU RA DEU KEM DO LECH CHUAN")
    print("=" * 78)
    grep(repo, "selfdrive/modeld/parse_model_outputs.py", "pred_std = safe_exp")
    T = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
    ys = np.array([list(e.modelV2.position.yStd) for e in mv])
    print(f"\n    {'chi so':>7} {'thoi diem':>11} {'yStd trung vi':>15}")
    for i in (6, 10, 14, 18, 32):
        print(f"    {i:>7} {T[i]:>10.2f}s {np.median(ys[:, i]):>14.3f} m")
    print("\n    CANH BAO: luoi thoi gian la ham bac hai nen khong co moc tron.")
    print(f"    Chi so 16 KHONG phai 2 giay ma la {T[16]:.1f} giay.")
    print("    Moc 1 / 2 / 3 giay gan nhat la chi so 10 / 14 / 18.")
    print("\n    Quy dao KHONG dung da gia thuyet trong ban model nay:")
    grep(repo, "selfdrive/modeld/parse_model_outputs.py", "parse_mdn('plan'")
    grep(repo, "selfdrive/modeld/parse_model_outputs.py", "lead_in_N, lead_out_N")

    print("\n" + "=" * 78)
    print("1.5  LENH LAI LA GIA TOC NGANG")
    print("=" * 78)
    grep(repo, "selfdrive/modeld/modeld.py", "/ (max(1.0, v_ego))**2")
    st, v = lg.series("carState", "vEgo")
    e0 = mv[0]
    k = float(e0.modelV2.action.desiredCurvature)
    u = float(np.interp(e0.logMonoTime, st, v))
    print(f"\n    Khung hinh dau: desiredCurvature = {k:+.5f} 1/m, v = {u:.2f} m/s")
    print(f"    -> gia toc ngang tuong ung = k*v^2 = {k*u**2:+.3f} m/s2")

    print("\n" + "=" * 78)
    print("1.6  DO TRE VA TAM NHIN CUA LENH LAI")
    print("=" * 78)
    grep(repo, "selfdrive/modeld/modeld.py", "lat_action_t = lat_delay")
    grep(repo, "selfdrive/modeld/modeld.py", "LAT_SMOOTH_SECONDS =")
    cp = lg.first("carParams")
    d = lg.first("liveDelay")
    me = np.array([e.modelV2.modelExecutionTime for e in mv]) * 1000
    if d:
        print(f"\n    liveDelay.lateralDelay (hoc truc tuyen) = {d.lateralDelay:.4f} s"
              f"  [trang thai {d.status}, hieu chuan {d.calPerc}%]")
    else:
        print("\n    KHONG DOC DUOC liveDelay — gan nhu chac chan dang dung SCHEMA SAI.")
        print("    Ban tin nay chi hien ra khi doc bang schema cua chinh ban fork.")
        print("    Tai lieu ghi 0,326 s va tam nhin lenh lai 0,401 s la lay tu day.")
    print(f"    carParams.steerActuatorDelay (cau hinh)  = {cp.steerActuatorDelay:.3f} s")
    print(f"    frame_delay + action_delay               = 0.075 s")
    if d:
        print(f"    => lat_action_t  = {d.lateralDelay + 0.075:.3f} s"
              f"   (o 60 km/h la {(d.lateralDelay+0.075)*60/3.6:.1f} m phia truoc)")
    print(f"    => long_action_t = {cp.longitudinalActuatorDelay + 0.3 + 0.075:.3f} s")
    print(f"\n    modelExecutionTime: trung vi {np.median(me):.1f} ms, P90 {np.percentile(me,90):.1f} ms"
          f"  (ngan sach 50 ms)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    main(os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]))
