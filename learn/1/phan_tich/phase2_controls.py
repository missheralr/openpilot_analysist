"""
phase2_controls.py — Sinh ra moi con so cua Phase 2 trong tai lieu (khoi controlsd).

    python phase2_controls.py <rlog.zst> <repo_fork>

  2.1  xac dinh xe dung dieu khien GOC hay MO-MEN
  2.2  chuoi bien doi 5 buoc, kem thien lech trai rieng cua VinFast
  2.3  tach goc co ban va phan PI, do xem xe thuc hien duoc bao nhieu phan tram
"""
import os
import sys
import math
import numpy as np
from collections import Counter


def grep(repo, rel, pattern, n=3):
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
    cp = lg.first("carParams")
    ctl = lg.get("controlsState")

    print("\n" + "=" * 78)
    print("2.1  XE NAY DIEU KHIEN THEO GOC HAY MO-MEN?")
    print("=" * 78)
    print(f"    carParams.steerControlType = {cp.steerControlType}")
    print(f"    carParams.lateralTuning    = {cp.lateralTuning.which()}")
    print(f"    Kieu bo dieu khien trong log: {dict(Counter(e.controlsState.lateralControlState.which() for e in ctl))}")
    cc0 = lg.first("carControl")
    print(f"    actuators.torque o khung dau = {cc0.actuators.torque}")
    grep(repo, "selfdrive/controls/controlsd.py", "SteerControlType.angle")

    print("\n" + "=" * 78)
    print("2.2  THIEN LECH TRAI RIENG CHO VINFAST")
    print("=" * 78)
    grep(repo, "selfdrive/controls/controlsd.py", 'self.CP.brand == "vinfast"', n=2)
    grep(repo, "selfdrive/controls/controlsd.py", "curv_scale = 0.002")

    cc_t = np.array([e.logMonoTime for e in lg.get("carControl")])
    lat = np.array([bool(e.carControl.latActive) for e in lg.get("carControl")])
    st, v = lg.series("carState", "vEgo")
    bias, kk = [], []
    for e in lg.get("modelV2"):
        t = e.logMonoTime
        if not lat[max(int(np.searchsorted(cc_t, t, side="right")) - 1, 0)]:
            continue
        k = float(e.modelV2.action.desiredCurvature)
        u = float(np.interp(t, st, v)) * 3.6
        if u >= 60:
            o = 0.0
        else:
            om = -0.0015 if u < 20 else (-0.001 if u < 40 else -0.0005 * math.exp(-((u - 40) / 20) * 4.0))
            o = om * math.exp(-abs(k) / 0.002)       # he so lam mo khi dang vao cua
        bias.append(abs(o)); kk.append(abs(k))
    bias, kk = np.array(bias), np.array(kk)
    print(f"\n    n = {len(bias)} khung hinh dang lai tu dong")
    print(f"    {'tinh huong':28s} {'n':>5} {'thien lech':>12} {'so voi lenh':>13}")
    for lo, hi, lbl in ((0, 0.0005, "gan nhu di thang"), (0.0005, 0.002, "duong cong nhe"), (0.002, 9, "dang vao cua")):
        m = (kk >= lo) & (kk < hi)
        if m.sum() > 10:
            print(f"    {lbl:28s} {m.sum():>5} {np.median(bias[m]):>12.5f} "
                  f"{np.median(bias[m]/np.maximum(kk[m],1e-6)):>12.0%}")
    print(f"    {'TOAN BO':28s} {len(bias):>5} {np.median(bias):>12.5f} "
          f"{np.median(bias/np.maximum(kk,1e-6)):>12.0%}")
    kmax = np.median(bias[kk < 0.0005]) if (kk < 0.0005).sum() else float("nan")
    u22 = 22 / 3.6
    print(f"\n    Y nghia vat ly tren duong thang: {kmax:.5f} 1/m o 22 km/h")
    print(f"      -> gia toc ngang {kmax*u22**2:.3f} m/s2, lech {0.5*kmax*u22**2*9:.2f} m sau 3 giay")

    print("\n" + "=" * 78)
    print("2.3  GIOI HAN ISO VA THAM SO XE HOC DUOC")
    print("=" * 78)
    grep(repo, "selfdrive/controls/lib/drive_helpers.py", "MAX_LATERAL_JERK =")
    grep(repo, "selfdrive/controls/lib/drive_helpers.py", "MAX_LATERAL_ACCEL_NO_ROLL =")
    print(f"\n    {'toc do':>9} {'do cong toi da':>16} {'ban kinh nho nhat':>19}")
    for kmh in (22, 60, 100):
        u = kmh / 3.6
        kmaxv = 3.0 / u ** 2
        print(f"    {kmh:>6} km/h {kmaxv:>15.4f} {1/kmaxv:>18.0f} m")
    ten_tham_so = lg.pick("liveParameters", "vehicleParameters")   # hai schema dat ten khac nhau
    P = lg.first(ten_tham_so)
    print(f"\n    (doc tu ban tin '{ten_tham_so}')")
    print(f"    {'tham so':26s} {'cau hinh goc':>14} {'hoc duoc':>11}")
    print(f"    {'steerRatio':26s} {cp.steerRatio:>14.2f} {P.steerRatio:>11.2f}")
    print(f"    {'stiffnessFactor':26s} {1.0:>14.2f} {P.stiffnessFactor:>11.2f}")
    print(f"    {'angleOffsetDeg':26s} {0.0:>14.2f} {P.angleOffsetDeg:>11.2f}")

    print("\n" + "=" * 78)
    print("2.4  GOC CO BAN vs PHAN PI — VA XE THUC HIEN DUOC BAO NHIEU")
    print("=" * 78)
    m_, wb = cp.mass, cp.wheelbase
    aF = cp.centerToFront; aR = wb - aF
    cF0, cR0, chi = cp.tireStiffnessFront, cp.tireStiffnessRear, cp.steerRatioRear

    def cfac(u, s):
        """Ban sao cua VehicleModel.curvature_factor — he so quy doi goc banh xe sang do cong."""
        cF, cR = s * cF0, s * cR0
        sf = -(m_ * (cF * aF - cR * aR) / (wb ** 2)) / (cF * cR)
        return (1. - chi) / (1. - sf * u ** 2) / wb

    ff, cmd, act = [], [], []
    for e in ctl:
        s_ = e.controlsState.lateralControlState
        if s_.which() != "angleState" or not s_.angleState.active:
            continue
        cs = lg.at("carState", e.logMonoTime); pp = lg.at(ten_tham_so, e.logMonoTime)
        u = max(cs.vEgo, 0.1)
        ff.append(-(math.degrees(e.controlsState.desiredCurvature * pp.steerRatio / cfac(u, pp.stiffnessFactor)) + pp.angleOffsetDeg))
        cmd.append(s_.angleState.steeringAngleDesiredDeg)
        act.append(cs.steeringAngleDeg)
    ff, cmd, act = np.array(ff), np.array(cmd), np.array(act)
    print(f"    n = {len(ff)} khung hinh dang lai tu dong")
    print(f"    goc co ban tu do cong : trung vi |.| = {np.median(np.abs(ff)):.2f} do")
    print(f"    phan PI cong them     : trung vi |.| = {np.median(np.abs(cmd-ff)):.2f} do,"
          f" P90 = {np.percentile(np.abs(cmd-ff),90):.2f} do")
    print(f"    cham tran 22 do       : {100*np.mean(np.abs(cmd-ff)>21.5):.1f}% so khung hinh")
    print(f"\n    vo lang thuc = k x LENH DAY DU : k = {np.polyfit(cmd,act,1)[0]:.3f}  (tuong quan {np.corrcoef(cmd,act)[0,1]:.3f})")
    print(f"    vo lang thuc = k x GOC CO BAN  : k = {np.polyfit(ff,act,1)[0]:.3f}  (tuong quan {np.corrcoef(ff,act)[0,1]:.3f})")
    print("\n    Kiem tra xem co phai do TRE khong — dich lenh ve truoc roi do lai:")
    for k in range(0, 41, 10):
        e_ = np.median(np.abs(cmd[:len(cmd)-k] - act[k:])) if k else np.median(np.abs(cmd - act))
        print(f"      dich {k*0.01:.2f}s: sai so {e_:.2f} do")
    print("    Sai so KHONG giam khi dich => khong phai tre, ma la thieu hut thuong truc.")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    main(os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]))
