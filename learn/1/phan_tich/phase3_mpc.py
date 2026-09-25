"""
phase3_mpc.py — Sinh ra moi con so cua Phase 3 trong tai lieu (ga-phanh, MPC).

    python phase3_mpc.py <rlog.zst> <repo_fork>

  3.2  bien dieu khien la do giat chu khong phai gia toc
  3.3  trong so ham muc tieu
  3.4  moi thu deu quy ve mot vat can, chon bang argmin
  3.5  cong thuc khoang cach mong muon + doi chieu voi thuc te
  3.6  bo giai va thoi gian chay
  3.7  mang chen vao bang phep min
"""
import os
import sys
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


MPC = "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
PLN = "selfdrive/controls/lib/longitudinal_planner.py"


def main(rlog, repo):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import oplog
    # Uu tien bien moi truong — de dung duoc ca khi schema nam rieng mot cho,
    # khong nhat thiet phai co ca repo ma nguon.
    oplog.CEREAL = os.environ.get("CEREAL_DIR") or os.path.join(repo, "cereal")
    oplog.OPENDBC = os.environ.get("OPENDBC_DIR") or os.path.join(repo, "opendbc_repo/opendbc/car")
    lg = oplog.Log(rlog)

    print("\n" + "=" * 78)
    print("3.2  TRANG THAI VA BIEN DIEU KHIEN")
    print("=" * 78)
    grep(repo, MPC, "model.x = vertcat")
    grep(repo, MPC, "model.u = vertcat")
    grep(repo, MPC, "N = 12")
    print("    -> bien dieu khien la DO GIAT; gia toc la trang thai nen khong the nhay bac")

    print("\n" + "=" * 78)
    print("3.3  TRONG SO HAM MUC TIEU")
    print("=" * 78)
    for k in ("X_EGO_OBSTACLE_COST", "X_EGO_COST", "V_EGO_COST", "A_EGO_COST",
              "A_CHANGE_COST", "J_EGO_COST", "DANGER_ZONE_COST", "LIMIT_COST", "LEAD_DANGER_FACTOR"):
        grep(repo, MPC, k + " =", n=1)
    grep(repo, MPC, "jerk_factor * a_change_cost", n=1)
    print("    -> hai trong so cuoi con nhan them he so theo che do lai (aggressive = 0,5)")

    print("\n" + "=" * 78)
    print("3.4  MOI THU DEU QUY VE MOT VAT CAN")
    print("=" * 78)
    grep(repo, MPC, "def get_stopped_equivalence_factor", n=2)
    grep(repo, MPC, "MPC_SOURCES[np.argmin")
    lp = [e.longitudinalPlan for e in lg.get("longitudinalPlan")]
    c = Counter(str(x.longitudinalPlanSource) for x in lp)
    print("\n    Nguon quyet dinh do tren log:",
          {k: f"{100*n/len(lp):.0f}%" for k, n in c.most_common()})

    print("\n" + "=" * 78)
    print("3.5  KHOANG CACH MONG MUON")
    print("=" * 78)
    grep(repo, MPC, "def get_safe_obstacle_distance", n=2)
    for k in ("COMFORT_BRAKE", "STOP_DISTANCE"):
        grep(repo, MPC, k + " =", n=1)
    pers = str(lg.first("selfdriveState").personality)
    TF = {"aggressive": 1.25, "standard": 1.45, "relaxed": 1.75}[pers]
    CB, SD = 2.5, 6.0
    print(f"\n    Che do lai tren xe: {pers}  ->  t_follow = {TF} s")
    print(f"    {'toc do':>9} {'tong':>9}   = {'phanh':>7} + {'tfollow':>8} + dung")
    for kmh in (10, 20, 30, 40, 60):
        v = kmh / 3.6
        print(f"    {kmh:>6} km/h {v**2/(2*CB)+TF*v+SD:>8.1f} m   = {v**2/(2*CB):>7.1f} + {TF*v:>8.1f} + {SD:.1f}")

    rows = []
    for e in lg.get("radarState"):
        R = e.radarState.leadOne
        present = getattr(R, "present", None) if hasattr(R, "present") else R.status
        if not present:
            continue
        cs = lg.at("carState", e.logMonoTime); cc = lg.at("carControl", e.logMonoTime)
        if not cc.longActive or cs.vEgo < 2:
            continue
        rows.append((R.dRel, cs.vEgo ** 2 / (2 * CB) + TF * cs.vEgo + SD))
    a = np.array(rows)
    print(f"\n    Doi chieu tren {len(a)} khung hinh dang giu ga, co xe truoc, v > 2 m/s:")
    print(f"      khoang cach THUC TE : {np.median(a[:,0]):.1f} m")
    print(f"      MPC MONG MUON       : {np.median(a[:,1]):.1f} m")
    print(f"      ty le               : {np.median(a[:,0]/a[:,1]):.2f}")

    print("\n" + "=" * 78)
    print("3.6  BO GIAI")
    print("=" * 78)
    grep(repo, MPC, "ACADOS_SOLVER_TYPE =", n=1)
    grep(repo, MPC, "qp_solver =", n=1)
    sv = np.array([x.solverExecutionTime for x in lp]) * 1000
    print(f"\n    Thoi gian giai do tren log: trung vi {np.median(sv):.2f} ms,"
          f" P99 {np.percentile(sv,99):.2f} ms  (ngan sach 50 ms)")

    print("\n" + "=" * 78)
    print("3.7  MANG NO-RON CHEN VAO BANG PHEP MIN")
    print("=" * 78)
    grep(repo, PLN, "min(output_a_target_e2e", n=1)
    print("    -> model chi duoc phep lam xe CHAM LAI, khong bao gio duoc ga manh hon MPC")
    print("\n    Cac lop gioi han cuoi:")
    for k in ("A_CRUISE_MAX_VALS", "A_CRUISE_MAX_BP", "ALLOW_THROTTLE_THRESHOLD"):
        grep(repo, PLN, k + " =", n=1)
    grep(repo, "opendbc_repo/opendbc/car/interfaces.py", "ACCEL_MAX =", n=1)
    grep(repo, "opendbc_repo/opendbc/car/interfaces.py", "ACCEL_MIN =", n=1)
    print(f"\n    aTarget do tren log: trung vi {np.median([x.aTarget for x in lp]):+.3f},"
          f" P10 {np.percentile([x.aTarget for x in lp],10):+.2f},"
          f" P90 {np.percentile([x.aTarget for x in lp],90):+.2f} m/s2")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    main(os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]))
