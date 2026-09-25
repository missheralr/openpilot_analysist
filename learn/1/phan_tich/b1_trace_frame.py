"""
BÀI 1 — Bám theo một frame đi qua toàn bộ hệ thống.

MỤC TIÊU: thấy tận mắt dữ liệu chảy từ camera → model → planner → lệnh lái,
trên MỘT frame duy nhất. Sau bài này bạn hiểu đường đi của dữ liệu rõ hơn
đọc 10 trang tài liệu.

CÁCH LÀM: chạy script này, rồi mở song song các file mã nguồn được nhắc trong
output. Mỗi con số script in ra đều có một dòng code sinh ra nó — tìm dòng đó.

    python b1_trace_frame.py <rlog.zst> [frame_id]

BÀI TẬP kèm theo (tự trả lời trước khi xem code):
  1. position.x có 33 điểm. Khoảng cách giữa chúng có đều nhau không? Vì sao?
  2. desiredCurvature và curvature khác nhau thế nào? Cái nào là lệnh, cái nào là thực tế?
  3. Vì sao modelV2 chạy 20Hz mà carState chạy 100Hz?
"""

import sys
import numpy as np
from oplog import Log


def trace(path, frame_id=None):
    lg = Log(path)

    mv_events = lg.get("modelV2")
    if not mv_events:
        print("Log nay khong co modelV2.")
        return

    # chọn frame: mặc định lấy frame giữa log, lúc xe đang chạy
    if frame_id is None:
        ev = mv_events[len(mv_events) // 2]
    else:
        ev = next((e for e in mv_events if e.modelV2.frameId == frame_id), None)
        if ev is None:
            print(f"Khong tim thay frameId={frame_id}")
            return

    m = ev.modelV2
    t = ev.logMonoTime

    print("=" * 66)
    print(f"BAM THEO FRAME  frameId={m.frameId}   logMonoTime={t}")
    print("=" * 66)

    # ── 1. ĐẦU VÀO: camera ────────────────────────────────────────────
    print("\n[1] CAMERA  (selfdrive/modeld/modeld.py doc frame tu day)")
    fm = lg.frame_map()
    inv = {v: k for k, v in fm.items()}
    vf = inv.get(m.frameId)
    print(f"    frameId thiet bi        : {m.frameId}")
    print(f"    chi so frame trong video: {vf}   <- KHAC nhau, map qua EncodeIndex")
    print(f"    thoi gian xu ly model   : {m.modelExecutionTime*1000:.1f} ms")

    # ── 2. Ý ĐỊNH: model TỰ ĐOÁN tài xế định làm gì ────────────────────
    print("\n[2] Y DINH MODEL TU DOAN  (meta.desireState — la DAU RA cua model)")
    print("    LUU Y: desireState la model TU DOAN tu hinh anh, KHONG phai lenh dua vao model.")
    print("    Lenh dua VAO model (desire) do desire_helper.py tao ra va KHONG duoc ghi vao log.")
    ds = list(m.meta.desireState)
    names = ["none", "turnLeft", "turnRight", "LC-Left", "LC-Right", "keepLeft", "keepRight"]
    top = int(np.argmax(ds[:7]))
    print(f"    Model doan: {names[top]} = {ds[top]:.3f}")
    print("    -> modeld.py dong ~430: xac suat doi lan lay tu day de quyet dinh khi nao doi lan XONG")

    # ── 3. ĐẦU RA: quỹ đạo ────────────────────────────────────────────
    print("\n[3] DAU RA — QUY DAO  (parse_model_outputs.py -> fill_model_msg.py)")
    p = m.position
    x, y, ts = list(p.x), list(p.y), list(p.t)
    print(f"    So diem: {len(x)}   (constants.py: IDX_N = 33)")
    print(f"    {'i':>3} {'t(s)':>7} {'x(m)':>8} {'y(m)':>8} {'yStd':>8}")
    for i in [0, 4, 8, 16, 24, 32]:
        if i < len(x):
            print(f"    {i:>3} {ts[i]:>7.2f} {x[i]:>8.2f} {y[i]:>8.2f} {list(p.yStd)[i]:>8.4f}")
    print(f"    LUU Y: y duong = BEN PHAI (device frame), khong phai ben trai.")
    print(f"    yStd tang tu {list(p.yStd)[0]:.4f} -> {list(p.yStd)[-1]:.4f}: model TU KHAI do bat dinh")

    # ── 4. ĐẦU RA: nhận thức làn ─────────────────────────────────────
    print("\n[4] DAU RA — NHAN THUC LAN")
    probs = list(m.laneLineProbs)
    lbl = ["xa-trai", "TRAI", "PHAI", "xa-phai"]
    for i, pr in enumerate(probs):
        bar = "#" * int(pr * 30)
        print(f"    {lbl[i]:>8}: {pr:.3f} {bar}")
    print(f"    roadEdgeStds (thap=tin cay): {[round(s,2) for s in m.roadEdgeStds]}")

    # ── 5. ĐẦU RA: xe phía trước ─────────────────────────────────────
    if len(m.leadsV3):
        l = m.leadsV3[0]
        print("\n[5] DAU RA — XE PHIA TRUOC")
        print(f"    prob={l.prob:.3f}  x[0]={list(l.x)[0]:.1f}m  y[0]={list(l.y)[0]:.2f}m  v[0]={list(l.v)[0]:.1f}m/s")

    # ── 6. PLANNER + ĐIỀU KHIỂN ──────────────────────────────────────
    print("\n[6] LENH DIEU KHIEN  (controlsd.py -> latcontrol_*.py)")
    cst = lg.at("controlsState", t)
    cs = lg.at("carState", t)
    cc_ev = lg.get("carControl")
    engaged = lg.is_engaged_at(t)
    if cst:
        print(f"    desiredCurvature : {cst.desiredCurvature:+.5f} 1/m   <- LENH model muon")
        print(f"    curvature        : {cst.curvature:+.5f} 1/m   <- xe THUC dat duoc")
        print(f"    bo dieu khien    : {cst.lateralControlState.which()}")
    if cs:
        print(f"    vEgo             : {cs.vEgo:.2f} m/s ({cs.vEgo*3.6:.0f} km/h)")
        print(f"    steeringAngleDeg : {cs.steeringAngleDeg:+.2f} deg  <- goc vo lang THUC")
    print(f"    Openpilot dang lai: {'CO' if engaged else 'KHONG (nguoi lai)'}")
    if not engaged:
        print("    -> Khi KHONG engaged, desiredCurvature duoc gan = curvature,")
        print("       nen sai so luon = 0. Chi so nay chi co nghia khi dang engaged.")

    # ── 7. TỰ ĐÁNH GIÁ ───────────────────────────────────────────────
    print("\n[7] MODEL TU DANH GIA RUI RO  (constants.py class Meta)")
    dp = m.meta.disengagePredictions
    print(f"    Moc thoi gian (giay)       : {list(dp.t)}")
    print(f"    Xac suat nguoi gianh lai   : {[round(v,3) for v in list(dp.steerOverrideProbs)]}")
    print(f"    Xac suat phanh gap         : {[round(v,4) for v in list(dp.brakeDisengageProbs)]}")

    print("\n" + "=" * 66)
    print("VIEC CAN LAM: mo cac file duoc nhac o tren, tim dong code sinh ra")
    print("tung con so. Ghi lai cho nao ban KHONG hieu — do la bai tiep theo.")
    print("=" * 66)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Dung: python b1_trace_frame.py <rlog.zst> [frame_id]")
        sys.exit(1)
    fid = int(sys.argv[2]) if len(sys.argv) > 2 else None
    trace(sys.argv[1], fid)
