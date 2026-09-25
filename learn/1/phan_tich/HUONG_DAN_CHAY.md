# Hướng dẫn chạy từ đầu

Làm tuần tự từ bước 0. Mỗi bước có cách kiểm tra đã xong chưa trước khi sang bước sau.

---

## Bước 0 — Cài thư viện

```bash
python3 --version          # cần 3.10 trở lên
pip3 install pycapnp zstandard numpy pandas scipy
```

Máy bạn không có lệnh `python`, chỉ có `python3`. Mọi lệnh dưới đây đều dùng `python3`.

Nếu muốn gõ `python` cho gọn:

```bash
sudo apt install python-is-python3
```

Kiểm tra xong chưa:

```bash
python3 -c "import capnp, zstandard, numpy, pandas, scipy; print('du thu vien')"
```

---

## Cách nhanh nhất — một lệnh duy nhất

```bash
cd <thư-mục-chứa-các-script>
python3 chay.py
```

Không tham số. Script tự tìm file rlog, tự tìm schema, tự thử đọc bằng từng bộ
schema tìm được rồi chọn bộ tốt nhất, tự tìm mã nguồn để trích dẫn, chạy cả 5
Phase và lưu vào thư mục `ket_qua/`.

Nếu nó chọn nhầm file rlog thì chỉ định tay:

```bash
python3 chay.py --rlog ~/openpilot/data/segment_22/rlog.zst
```

Chỉ muốn xem nó dò được gì mà chưa chạy:

```bash
python3 chay.py --chi-tim
```

Phần còn lại của tài liệu này giải thích từng bước thủ công, dùng khi cần can
thiệp sâu hơn hoặc khi `chay.py` báo lỗi.

---

## Bước 1 — Cần những gì, và không cần gì

Không bắt buộc phải clone cả repo fork. Có ba mức, mỗi mức mở thêm một phần:

| Mức | Cần gì | Chạy được gì |
|---|---|---|
| 1 | **schema capnp** — 5 tệp, 124 KB | toàn bộ **số liệu đo từ log** ở cả 5 Phase |
| 2 | thêm mã nguồn bản fork | in kèm dòng mã nguồn sinh ra từng con số |
| 3 | thêm `python3.12` | lấy hằng số trong các tệp `.pyc` (bảng PID của VF6) |

Mức 1 là đủ để ra mọi con số trong tài liệu. Hai mức sau chỉ để đối chiếu mã nguồn.

Chạy script này để biết máy bạn đang có gì:

```bash
cd <thư-mục-chứa-các-script>
python3 tim_schema.py <đường-dẫn-rlog>
```

Nó quét cả thư mục nhà, thử đọc rlog bằng từng bộ schema tìm được, rồi in ra
chính xác mấy dòng `export` cần dán vào terminal. Ví dụ kết quả:

```
-- /home/bop/openpilot/cereal
   car.capnp: /home/bop/openpilot/opendbc_repo/opendbc/car   [BAN FORK]
   DOC DUOC: 106434 ban tin | xe VINFAST_VF6 | safetyModel vinfastVf6 | liveDelay 240
   -> bo NAY dung
```

Nếu nó báo tìm được schema của **bản gốc comma** thay vì bản fork, vẫn chạy được:
mọi con số đo từ log đều đúng, chỉ có vài **tên** bị sai (`safetyModel` hiện là
`volvo` thay vì `vinfastVf6`) và ba bản tin `liveDelay`, `liveTracks`,
`liveParameters` bị đọc thành rỗng. Khi đó mục 1.6 của Phase 1 và mục 4.1 của
Phase 4 sẽ thiếu số liệu.

### Nếu chưa có schema nào

Tải đúng 5 tệp, không cần clone:

```bash
BASE=https://raw.githubusercontent.com/qmpilot-vn/openpilot/vf-release-c4
mkdir -p ~/vf_schema/cereal/include ~/vf_schema/opendbc/car
cd ~/vf_schema
curl -fL $BASE/cereal/log.capnp         -o cereal/log.capnp
curl -fL $BASE/cereal/custom.capnp      -o cereal/custom.capnp
curl -fL $BASE/cereal/deprecated.capnp  -o cereal/deprecated.capnp
curl -fL $BASE/cereal/include/c++.capnp -o cereal/include/c++.capnp
curl -fL $BASE/opendbc_repo/opendbc/car/car.capnp -o opendbc/car/car.capnp
ln -sf ../opendbc/car/car.capnp cereal/car.capnp
```

Repo là private thì `curl` báo 404. Khi đó copy từ chính thiết bị trên xe:

```bash
scp -r comma@<ip-xe>:/data/openpilot/cereal ~/vf_schema/cereal
scp comma@<ip-xe>:/data/openpilot/opendbc_repo/opendbc/car/car.capnp ~/vf_schema/opendbc/car/
```

## Bước 2 — Đặt biến và thử đọc log

```bash
cd <thư-mục-chứa-các-script>
export RLOG=<đường-dẫn-rlog>
export CEREAL_DIR=<thư-mục-cereal>
export OPENDBC_DIR=<thư-mục-chứa-car.capnp>
export OPENPILOT_DIR=<thư-mục-gốc-repo>     # chỉ cần nếu có mức 2

python3 oplog.py $RLOG
```

Phải in ra danh sách bản tin, tốc độ, tỉ lệ thời gian lái tự động.

Ba biến `CEREAL_DIR`, `OPENDBC_DIR`, `OPENPILOT_DIR` được mọi script đọc. Đặt
một lần đầu phiên làm việc là dùng được hết. Nếu mở terminal mới thì phải đặt lại.

## Bước 3 — Chạy năm script Phase

Mỗi script in ra đúng số liệu của một Phase trong tài liệu.

```bash
python3 phase1_modeld.py     $RLOG $OPENPILOT_DIR
python3 phase2_controls.py   $RLOG $OPENPILOT_DIR
python3 phase3_mpc.py        $RLOG $OPENPILOT_DIR
python3 phase4_radard.py     $RLOG $OPENPILOT_DIR
python3 phase5_selfdrived.py $RLOG $OPENPILOT_DIR
```

Chỉ có mức 1 (schema, không có mã nguồn) thì truyền gì cũng được vào tham số thứ
hai — script vẫn in đủ số liệu, chỉ bỏ qua phần trích mã nguồn và ghi rõ
`(khong thay ...)`:

```bash
python3 phase2_controls.py $RLOG khong-co
```

Phase 4 và 5 in thêm được mã nguồn gốc nếu bạn có repo của comma:

```bash
git clone https://github.com/commaai/openpilot ~/op-goc      # nếu chưa có
python3 phase4_radard.py     $RLOG $OPENPILOT_DIR ~/op-goc
python3 phase5_selfdrived.py $RLOG $OPENPILOT_DIR ~/op-goc
```

Lưu lại để đọc sau:

```bash
for p in 1_modeld 2_controls 3_mpc 4_radard 5_selfdrived; do
  python3 phase$p.py $RLOG $FORK > ket_qua_phase$p.txt 2>&1
done
```

---

## Bước 4 — Lấy hằng số từ các module bị đóng gói

Bảng PID của VF6 và hơn 150 hằng số của radard nằm trong file `.pyc`, không đọc
trực tiếp được.

```bash
python3.12 decompile_consts.py $OPENPILOT_DIR
```

Bắt buộc `python3.12` vì các file này build cho phiên bản đó. Nếu máy chưa có:

```bash
sudo apt install python3.12
```

Xem riêng một module:

```bash
python3.12 decompile_consts.py $OPENPILOT_DIR latcontrol_angle_impl
```

---

## Bước 5 — Chạy pipeline đo trên toàn route

Bốn bước trên chạy trên một segment để giải thích cơ chế. Bước này chạy trên cả
route và sinh ra CSV để phân tích thống kê.

**Luôn chạy selftest trước.** Nó gài lỗi vào dữ liệu rồi kiểm tra xem phép đo có
phát hiện được không. Không đủ PASS thì đừng tin kết quả.

```bash
python3 selftest.py      $RLOG        # phải 14 PASS
python3 selftest_long.py $RLOG        # phải 14 PASS
```

Nếu báo `Khong thay log.capnp` thì bạn chưa đặt `OPENPILOT_DIR` ở bước 2.

Rồi chạy pipeline, trỏ vào **thư mục chứa các segment**, không phải file rlog:

```bash
python3 lane_analysis.py ~/learn/data 2>&1 | tee ketqua_lane.txt
python3 long_analysis.py ~/learn/data 2>&1 | tee ketqua_long.txt
```

Chạy thử nhanh vài segment trước khi chạy hết:

```bash
python3 lane_analysis.py ~/learn/data --max-seg 3
```

Kết quả sinh ra:

| Tệp | Nội dung |
|---|---|
| `00_inventory.csv` | mỗi segment: km, % lái tự động, chất lượng dữ liệu |
| `02_prediction.csv` | mỗi khung hình: sai số dự đoán quỹ đạo |
| `03_keeping.csv` | mỗi cửa sổ 5 giây: chỉ số giữ làn |
| `04_events.csv` | mỗi lần người can thiệp + dấu hiệu 3 giây trước |
| `05_envelope.csv` | bảng tổng hợp theo điều kiện |
| `05_clips.csv` | đoạn tệ nhất kèm frame_id để xem lại bằng video |
| `L0`…`L7_*.csv` | tương tự cho phần ga-phanh |

---

## Bước 6 — Đối chiếu với tài liệu

```bash
python3 verify_doc.py $RLOG $OPENPILOT_DIR ~/op-goc
```

Mở lại log và mở lại mã nguồn, tính lại 81 con số rồi so với con số ghi trong
tài liệu. Kết quả mong đợi: `81 KHOP | 0 LECH`.

---

## Gặp lỗi thường gặp

| Lỗi | Nguyên nhân | Cách sửa |
|---|---|---|
| `Command 'python' not found` | máy chỉ có `python3` | dùng `python3` |
| `can't open file ...` | đang đứng sai thư mục | `cd ~/learn/break/phan_tich` trước |
| `can't find '__main__' module in '.../phan_tich/'` | có dấu cách thừa giữa đường dẫn và tên file | bỏ dấu cách |
| `Khong thay log.capnp` | đường dẫn repo fork sai | làm lại bước 1b |
| `IndexError` khi decompile | sai phiên bản Python | dùng `python3.12` |
| `ModuleNotFoundError: capnp` | thiếu thư viện | làm lại bước 0 |
| Tên bản tin lạ, thiếu `liveDelay` | đang đọc bằng schema của bản gốc | truyền repo **fork**, không phải repo comma |
