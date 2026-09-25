"""
oplog.py — Bộ công cụ nền để đọc rlog của Openpilot.

Đây là module bạn import trong MỌI bài tập của lộ trình. Viết một lần, dùng mãi.
Không phụ thuộc LogReader (kéo theo nhiều dependency nặng), chỉ cần pycapnp + zstandard.

CÁCH DÙNG:
    from oplog import Log
    lg = Log("~/openpilot/data/segment_22/rlog.zst")

    lg.count()                      # đếm mọi loại bản tin
    lg.get("modelV2")               # lấy list bản tin theo tên
    lg.first("modelV2")             # lấy bản tin đầu tiên
    lg.series("carState", "vEgo")   # lấy (thời gian, giá trị) theo thời gian
    lg.at("carState", t)            # lấy bản tin gần nhất với mốc thời gian t

SETUP (chạy 1 lần):
    pip install pycapnp zstandard numpy
"""

import os
import sys
import bz2
from collections import Counter

import numpy as np

try:
    import capnp
    import zstandard as zstd
except ImportError:
    print("Thieu thu vien. Chay: pip install pycapnp zstandard numpy")
    sys.exit(1)


# ── ĐƯỜNG DẪN TỚI MÃ NGUỒN OPENPILOT ────────────────────────────────────
# Chỉ cần trỏ OPENPILOT_DIR vào thư mục gốc của repo (bản fork hoặc bản comma),
# script tự dò xem cereal/ và opendbc nằm ở đâu bên trong:
#
#     export OPENPILOT_DIR=~/qmpilot
#
# Hoặc sửa thẳng biến OPENPILOT bên dưới.
OPENPILOT = os.environ.get("OPENPILOT_DIR") or os.path.expanduser("~/openpilot")


def _find(root, *candidates):
    """Thử lần lượt các vị trí có thể, trả về cái đầu tiên tồn tại."""
    for c in candidates:
        p = os.path.join(root, c)
        if os.path.isdir(p):
            return p
    return os.path.join(root, candidates[0])


# fork để cereal ngay trong repo; bản comma clone về có thể lồng thêm một cấp
CEREAL = os.environ.get("CEREAL_DIR") or _find(OPENPILOT, "cereal", "openpilot/cereal")
OPENDBC = os.environ.get("OPENDBC_DIR") or _find(
    OPENPILOT, "opendbc_repo/opendbc/car", "openpilot/opendbc_repo/opendbc/car", "opendbc/car")
# ─────────────────────────────────────────────────────────────────────────


def _load_schema():
    capnp.remove_import_hook()
    if not os.path.isfile(os.path.join(CEREAL, "log.capnp")):
        print(f"Khong thay log.capnp trong {CEREAL}")
        print("Dat bien moi truong cho dung repo, vi du:")
        print("    export OPENPILOT_DIR=~/qmpilot")
        sys.exit(1)
    if not os.path.isfile(os.path.join(OPENDBC, "car.capnp")):
        print(f"Khong thay car.capnp trong {OPENDBC}")
        print("Can repo opendbc. Chay trong thu muc ~/openpilot:")
        print("  git submodule update --init --depth 1 opendbc_repo")
        sys.exit(1)
    return capnp.load(os.path.join(CEREAL, "log.capnp"),
                      imports=[CEREAL, OPENDBC, "."])


_SCHEMA = None


def schema():
    global _SCHEMA
    if _SCHEMA is None:
        # Nếu code của openpilot đã được import (vd. để so với class gốc), nó đã nạp
        # log.capnp rồi — dùng lại, vì capnp không cho nạp cùng một schema hai lần.
        op_cereal = sys.modules.get("openpilot.cereal")
        _SCHEMA = op_cereal.log if op_cereal is not None else _load_schema()
    return _SCHEMA


def _decompress(raw: bytes) -> bytes:
    if raw.startswith(b"BZh9"):
        return bz2.decompress(raw)
    if raw.startswith(b"\x28\xB5\x2F\xFD"):
        with zstd.ZstdDecompressor().stream_reader(raw) as r:
            return r.read()
    return raw


class Log:
    """Một rlog đã nạp sẵn vào bộ nhớ, truy cập nhanh theo tên bản tin."""

    def __init__(self, path):
        path = os.path.expanduser(path)
        with open(path, "rb") as f:
            data = _decompress(f.read())
        self.path = path
        self.events = list(schema().Event.read_multiple_bytes(data))
        self._by_type = {}
        for e in self.events:
            try:
                w = e.which()
            except Exception:
                continue
            self._by_type.setdefault(w, []).append(e)
        for lst in self._by_type.values():
            lst.sort(key=lambda e: e.logMonoTime)

    # ── truy cập cơ bản ──────────────────────────────────────────────
    def count(self, top=None):
        """Đếm số bản tin theo từng loại. count(20) = 20 loại nhiều nhất."""
        c = Counter({k: len(v) for k, v in self._by_type.items()})
        return c.most_common(top) if top else dict(c)

    def types(self):
        return sorted(self._by_type)

    def get(self, name):
        """Trả về list các EVENT (còn nguyên logMonoTime) của loại `name`."""
        return self._by_type.get(name, [])

    def payloads(self, name):
        """Trả về list phần NỘI DUNG (bỏ vỏ event) — tiện khi không cần thời gian."""
        return [getattr(e, name) for e in self.get(name)]

    def first(self, name):
        """Bản tin đầu tiên, hoặc None."""
        lst = self.get(name)
        return getattr(lst[0], name) if lst else None

    # ── làm việc theo thời gian ──────────────────────────────────────
    def times(self, name):
        return np.array([e.logMonoTime for e in self.get(name)], dtype=np.int64)

    def series(self, name, field):
        """(thoi_gian, gia_tri) của một trường vô hướng. Ví dụ: series('carState','vEgo')"""
        evs = self.get(name)
        t = np.array([e.logMonoTime for e in evs], dtype=np.int64)
        v = np.array([getattr(getattr(e, name), field) for e in evs])
        return t, v

    def pick(self, *names):
        """Tra ve ten ban tin dau tien co that trong log.

        Cung mot du lieu nhung hai ban schema dat ten khac nhau: liveParameters cua
        ban fork bi ban goc doc thanh vehicleParameters. Dung ham nay de script chay
        duoc voi ca hai."""
        for n in names:
            if self.get(n):
                return n
        return names[0]

    def at(self, name, mono_time):
        """Bản tin loại `name` gần nhất với mốc thời gian đã cho."""
        evs = self.get(name)
        if not evs:
            return None
        t = self.times(name)
        return getattr(evs[int(np.argmin(np.abs(t - mono_time)))], name)

    # ── đồng bộ frame video ──────────────────────────────────────────
    def frame_map(self, cam="narrowRoadEncodeIdx"):
        """{chi_so_frame_video: frameId thiet bi}.
        LƯU Ý: modelV2.frameId KHÔNG phải chỉ số frame khi trích bằng ffmpeg/cv2.
        Phải map qua EncodeIndex.segmentId — đây chính là chỉ số frame video."""
        return {e_.segmentId: e_.frameId
                for e_ in (getattr(e, cam) for e in self.get(cam))}

    def model_by_frame_id(self):
        """{frameId: ban tin modelV2}"""
        return {e.modelV2.frameId: e.modelV2 for e in self.get("modelV2")}

    # ── các nhãn điều kiện hay dùng ──────────────────────────────────
    def engaged_mask(self):
        """(thoi_gian, dang_lai_tu_dong) — Openpilot có đang điều khiển lái không."""
        evs = self.get("carControl")
        t = np.array([e.logMonoTime for e in evs], dtype=np.int64)
        v = np.array([bool(e.carControl.latActive) for e in evs])
        return t, v

    def is_engaged_at(self, mono_time):
        t, v = self.engaged_mask()
        if len(t) == 0:
            return False
        return bool(v[int(np.argmin(np.abs(t - mono_time)))])

    def brightness(self):
        """Chỉ số ánh sáng từ auto-exposure của camera. Giá trị nhỏ = trời sáng.
        Dùng để gắn nhãn ngày/đêm mà không cần giải mã video."""
        cams = [c for c in ("narrowRoadCameraState", "roadCameraState",
                            "wideRoadCameraState") if self.get(c)]
        if not cams:
            return None
        st = self.payloads(cams[0])
        return np.array([c.exposureValPercent for c in st])


def describe(path):
    """In nhanh tổng quan một rlog — chạy đầu tiên khi gặp log lạ."""
    lg = Log(path)
    print(f"File   : {lg.path}")
    print(f"Events : {len(lg.events)}")
    print("\n15 loai ban tin nhieu nhat:")
    for k, n in lg.count(15):
        print(f"  {k:28s} {n:6d}")

    cs = lg.get("carState")
    if cs:
        _, v = lg.series("carState", "vEgo")
        print(f"\nToc do : median={np.median(v):.1f} m/s ({np.median(v)*3.6:.0f} km/h)"
              f"  max={v.max():.1f} m/s ({v.max()*3.6:.0f} km/h)")
    _, eng = lg.engaged_mask()
    if len(eng):
        print(f"Engaged: {100*eng.mean():.1f}% thoi gian Openpilot dieu khien lai")
    b = lg.brightness()
    if b is not None:
        print(f"Anh sang (exposureValPercent): median={np.median(b):.4f}"
              f"  -> {'ban ngay' if np.median(b) < 1 else 'thieu sang / ban dem'}")
    return lg


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Dung: python oplog.py <duong_dan_rlog.zst>")
        sys.exit(1)
    describe(sys.argv[1])
