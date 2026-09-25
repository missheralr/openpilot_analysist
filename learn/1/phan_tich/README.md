# Code phân tích Openpilot trên VinFast VF6

Toàn bộ code đã dùng để suy ra các kết quả trong tài liệu
`openpilot_vf6_phan_tich_he_thong.docx`. Mỗi script in ra đúng những con số đã
đưa vào một mục cụ thể của tài liệu, kèm dòng mã nguồn sinh ra chúng.

## Cài đặt

```bash
pip install pycapnp zstandard numpy pandas scipy
```

Cần thêm hai thứ trên máy:

- **repo bản fork** mà xe đang chạy — `qmpilot-vn/openpilot`, nhánh `vf-release-c4`
- **repo bản gốc của comma** — chỉ cần cho Phase 4 và 5, vì fork đóng gói một số
  module thành bytecode và xoá mã nguồn

## Nền tảng

| Tệp | Vai trò |
|---|---|
| `oplog.py` | đọc rlog định dạng capnp mà không cần cài cả môi trường openpilot. Mọi script khác đều import tệp này |

**Bắt buộc:** đọc log của bản fork phải dùng schema của chính bản fork. Các script
ở đây tự trỏ `oplog.CEREAL` và `oplog.OPENDBC` vào repo fork bạn truyền vào. Nếu
dùng schema của bản gốc thì số liệu vẫn đúng nhưng **tên enum sẽ sai** và một số
bản tin **biến mất**. Xem mục 8.1 của tài liệu, hoặc chạy `phase5_selfdrived.py`
để thấy tận mắt.

## Script theo từng Phase

Tất cả dùng chung cú pháp:

```bash
python phaseN_xxx.py <rlog.zst> <repo_fork> [repo_goc_comma]
```

| Script | Sinh ra phần nào của tài liệu |
|---|---|
| `phase1_modeld.py` | Phase 1 — hai mạng, bộ đệm 25×512, desire dạng xung, bảng `yStd`, lệnh lái là gia tốc ngang, độ trễ học được 0,326 s |
| `phase2_controls.py` | Phase 2 — điều khiển theo góc, thiên lệch trái của VinFast, giới hạn ISO, tách góc cơ bản với phần PI, tỉ lệ thực hiện 44% |
| `phase3_mpc.py` | Phase 3 — biến điều khiển là độ giật, trọng số hàm mục tiêu, `argmin` chọn vật cản, bảng khoảng cách mong muốn, thời gian giải |
| `phase4_radard.py` | Phase 4 — radar cho gì, Kalman, ghép bằng tích ba xác suất, lệch −3,63 m, tỉ lệ phủ radar theo tốc độ |
| `phase5_selfdrived.py` | Phase 5 — máy trạng thái, hệ thống sự kiện, panda, vỏ rỗng an toàn VinFast, bẫy schema |
| `decompile_consts.py` | lấy hằng số từ các module bị đóng gói thành `.pyc` (bộ PID của VF6, hơn 150 hằng số của radard). **Phải chạy bằng `python3.12`** |

Ví dụ:

```bash
python phase2_controls.py ~/openpilot/data/segment_22/rlog.zst ~/qmpilot
python phase4_radard.py   ~/openpilot/data/segment_22/rlog.zst ~/qmpilot ~/openpilot-goc
python3.12 decompile_consts.py ~/qmpilot
```

## Pipeline đo toàn route

Khác với các script Phase ở trên (chạy trên một segment để giải thích cơ chế),
hai pipeline này chạy trên toàn bộ route và sinh ra CSV để phân tích thống kê.

| Tệp | Nội dung |
|---|---|
| `lane_analysis.py` | dự đoán quỹ đạo và giữ làn: 6 tầng, sinh 6 CSV |
| `selftest.py` | 14 phép kiểm tra cho `lane_analysis.py`. **Chạy trước, phải đủ 14 PASS** |
| `long_analysis.py` | ga-phanh và giữ khoảng cách: 8 tầng, sinh 7 CSV |
| `selftest_long.py` | 14 phép kiểm tra cho `long_analysis.py` |

```bash
python selftest.py ~/openpilot/data/segment_22/rlog.zst    # bắt buộc, 14 PASS
python lane_analysis.py ~/openpilot/data 2>&1 | tee ketqua.txt
```

Bộ selftest chia làm ba nhóm: kiểm tra công thức trên quỹ đạo tự chế có đáp án
biết trước, kiểm tra tính hợp lý trên dữ liệu thật, và **gài lỗi** (lệch 0,5 m,
lật dấu, xáo trộn frame, lệch thời gian) rồi đòi hỏi sai số phải tăng. Nhóm thứ
ba từng bắt được một lỗi lật dấu nghiêm trọng mà nhìn bảng kết quả không thể thấy.

## Tiện ích

| Tệp | Nội dung |
|---|---|
| `verify_doc.py` | đối chiếu 81 con số và trích dẫn trong tài liệu với log và mã nguồn |
| `download_routes.py` | tải rlog từ server qmpilot về đúng cấu trúc thư mục |
| `b1_trace_frame.py` | bám một khung hình đi qua cả sáu khối — bài khởi động dễ đọc nhất |

## Những gì code này KHÔNG chứng minh được

1. **Firmware panda thật trên xe** — mã đóng, không đọc được từ kho mã công khai.
2. **Logic bên trong các module `.pyc`** — `decompile_consts.py` lấy được hằng số
   và tên hàm, nhưng không phục hồi được toàn bộ luồng điều khiển.
3. **Các câu diễn giải trong tài liệu** — kiểu "phạt nặng như vậy để tránh rung
   giật" là suy luận, không phải sự kiện đo được. Đọc với tinh thần phản biện.

## Phạm vi dữ liệu

Phần lớn số liệu định lượng đến từ một segment 60 giây, nội thành, ban ngày,
khoảng 22 km/h. Chúng là chỉ dấu có bằng chứng chứ chưa phải kết luận cho mọi
điều kiện. Chưa có dữ liệu ban đêm và chưa có đoạn chạy liên tục trên 50 km/h.
