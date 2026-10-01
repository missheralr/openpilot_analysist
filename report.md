# Openpilot trên VinFast VF6

**Phân tích kiến trúc, thuật toán và cách hệ thống vận hành**

| | |
|---|---|
| Xe | VinFast VF6 — carFingerprint `VINFAST_VF6` |
| Phần mềm | `qmpilot-vn/openpilot`, nhánh `vf-release-c4` (fork của sunnypilot / comma openpilot) |
| Dữ liệu đối chứng | 1 route, 52 segment — 52 phút, 16,0 km nội thành Hà Nội. Phần phân tích lái dùng 45 segment đạt chuẩn kiểm định (45 phút, 15,3 km) |
| Phạm vi | 5 Phase: modeld → controlsd → longitudinal planner → radard → selfdrived & panda |
| Ngày | 25/09/2026 |

Mọi con số trong tài liệu này đều đo lại từ log thật của xe, không lấy từ tài liệu bên ngoài. Mỗi khẳng định về mã nguồn đều kèm tên tệp và số dòng để ai muốn cũng tra lại được.

---

## Mục lục

- [§1 — Kiến trúc tổng quan](#1-kiến-trúc-tổng-quan)
- [Phase 1 — modeld: từ ảnh đến lệnh lái](#phase-1--modeld-từ-ảnh-đến-lệnh-lái)
- [Phase 2 — controlsd: từ độ cong đến vô-lăng](#phase-2--controlsd-từ-độ-cong-đến-vô-lăng)
- [Phase 3 — longitudinal planner: bài toán tối ưu cho ga và phanh](#phase-3--longitudinal-planner-bài-toán-tối-ưu-cho-ga-và-phanh)
- [Phase 4 — radard: ghép radar với camera](#phase-4--radard-ghép-radar-với-camera)
- [Phase 5 — selfdrived và panda: ai quyết định, ai phủ quyết](#phase-5--selfdrived-và-panda-ai-quyết-định-ai-phủ-quyết)
- [§6 — Tổng hợp các bản vá riêng cho VinFast](#6-tổng-hợp-các-bản-vá-riêng-cho-vinfast)
- [§7 — Bảng số liệu đo được trên xe](#7-bảng-số-liệu-đo-được-trên-xe)
- [§8 — Kết quả đo phần lái trên toàn route](#8-kết-quả-đo-phần-lái-trên-toàn-route)
- [§9 — Kết quả đo ga-phanh trên toàn route](#9-kết-quả-đo-ga-phanh-trên-toàn-route)
- [§10 — Những điểm cần lưu ý khi đọc tài liệu này](#10-những-điểm-cần-lưu-ý-khi-đọc-tài-liệu-này)
- [§11 — Thuật ngữ](#11-thuật-ngữ)

---

## 1. Kiến trúc tổng quan

Openpilot là một chuỗi sáu khối nối tiếp, mỗi khối là một tiến trình riêng, nói chuyện với nhau qua bản tin định dạng capnp. Một vòng của chuỗi này là 50 mili-giây, theo nhịp 20 Hz của model.

Cần phân biệt nhịp chạy với độ trễ thật. Từ lúc ánh sáng chạm cảm biến đến lúc bánh xe thực sự đổi hướng lâu hơn nhiều: 0,326 giây. Con số đó không phải suy đoán, chính hệ thống tự học lấy (mục 1.6).

| Khối | Tần số | Nhiệm vụ | Đầu ra chính |
|---|---|---|---|
| Cảm biến | 20 / 104 / 100 Hz | camera, IMU, CAN | ảnh, gia tốc, tốc độ bánh xe |
| modeld | 20 Hz | mạng nơ-ron nhìn đường | quỹ đạo, vạch kẻ, xe trước, lệnh lái |
| radard | 20 Hz | ghép radar với thị giác | xe phía trước đã hợp nhất |
| longitudinal_planner | 20 Hz | tối ưu hoá ga-phanh | gia tốc mục tiêu |
| controlsd | 100 Hz | biến lệnh thành tín hiệu xe | góc vô-lăng, mức ga-phanh |
| selfdrived + panda | 100 Hz | quyết định cho phép, chặn vi phạm | bật/tắt, lọc bản tin CAN |

Có một ý tưởng xuyên suốt cả hệ thống, và theo tôi đây là điều đáng học nhất: mạng nơ-ron được trao quyền quyết định rất rộng, nhưng luôn bị kẹp giữa những lớp kiểm tra cổ điển kiểm chứng được. Mạng chỉ được phép phanh thêm chứ không được ga thêm. Độ cong quỹ đạo bị cắt theo giới hạn ISO. Góc lái còn bị firmware C trên panda soi lại lần nữa. Không lớp nào tin lớp trên nó.

---

## Phase 1 — modeld: từ ảnh đến lệnh lái

Cứ mỗi 50 mili-giây, modeld nhận ảnh, chạy mạng, xuất ra quỹ đạo và lệnh lái. Trên xe này thời gian chạy mạng có trung vị 25,8 ms trên toàn segment, tức khoảng một nửa ngân sách thời gian.

### 1.1. Hai mạng chứ không phải một

```python
# selfdrive/modeld/modeld.py:136
vision_output, on_policy_output = self.run_policy(...)
```

Mạng thị giác nhận ảnh và nhả ra một vector đặc trưng 512 chiều; nó không biết gì về quá khứ. Mạng chính sách nhận chuỗi các vector đó theo thời gian rồi quyết định phải làm gì. Tách đôi như vậy để phần nặng nhất chạy độc lập từng khung hình, còn phần suy luận theo thời gian thì nhẹ hơn nhiều.

### 1.2. Ký ức là một hàng đợi nhìn thấy được

```python
# selfdrive/modeld/compile_modeld.py:121
fb = policy_input_shapes['features_buffer']   # (1, 25, 512)

# selfdrive/modeld/modeld.py:98
self.frame_skip = MODEL_RUN_FREQ // MODEL_CONTEXT_FREQ   # 20 // 5 = 4
```

Bộ đệm giữ 25 vector đặc trưng, lấy mẫu mỗi 4 khung hình. Model chạy 20 Hz, lấy mẫu ở 5 Hz, nên 25 mẫu trải ra khoảng 5 giây quá khứ. Đây không phải trạng thái ẩn mờ ảo kiểu LSTM mà là một hàng đợi hiển ngôn. Muốn biết model đang nhớ gì thì nhìn thẳng vào đó.

Đầu vào ảnh có kích thước `(1, 12, 128, 256)`, tức hai khung hình, mỗi khung 6 kênh màu YUV. Hai khung này không liền nhau mà cách nhau 4 frame, tức 0,2 giây. Model cảm nhận chuyển động bằng cách so sánh hai ảnh cách nhau 200 mili-giây.

### 1.3. Lệnh đổi làn là một xung, không phải một trạng thái

```python
# selfdrive/modeld/modeld.py:123-125
inputs['desire_pulse'][0] = 0
self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99,
                                 inputs['desire_pulse'], 0)
self.prev_desire[:] = inputs['desire_pulse']
```

Khi cần đổi làn, hệ thống không giữ tín hiệu suốt quá trình mà chỉ bắn một xung duy nhất ở sườn lên. Comment ngay trên dòng đó giải thích lý do: model tự quyết định khi nào đổi làn xong.

Đây cũng là chỗ rất dễ hiểu nhầm khi đọc log. Trường `meta.desireState` là đầu ra của model, không phải lệnh đưa vào model.

### 1.4. Model xuất ra phân phối, không xuất ra con số

```python
# selfdrive/modeld/parse_model_outputs.py:52
pred_std = safe_exp(raw[:,:, n_values: 2*n_values])
```

Mỗi đại lượng được dự đoán hai lần: giá trị, và logarit của độ lệch chuẩn. Lấy hàm mũ để bảo đảm luôn dương. Mạng được huấn luyện để tối đa hoá xác suất của đáp án thật dưới phân phối đó, nên nó buộc phải học cách thừa nhận chỗ nào mình không chắc. Toàn bộ phần đánh giá độ tin cậy ở các Phase sau đều dựa trên đại lượng này.

| Tầm nhìn | Chỉ số trong mảng | yStd — trung vị cả segment |
|---|---|---|
| 0,35 giây | 6 | 0,008 m |
| 0,98 giây | 10 | 0,052 m |
| 1,91 giây | 14 | 0,131 m |
| 3,16 giây | 18 | 0,215 m |
| 10,0 giây | 32 | 1,129 m |

Lưới thời gian là hàm bậc hai nên không có mốc tròn 1, 2 hay 3 giây; muốn lấy các mốc đó phải chọn chỉ số gần nhất, lần lượt là 10, 14 và 18. Chỗ này bẫy người đọc log: chỉ số 16 không phải 2 giây mà là 2,5 giây.

Cơ chế `parse_mdn` còn hỗ trợ đa giả thuyết — mạng đưa ra nhiều phương án kèm trọng số softmax rồi chọn phương án nặng nhất thay vì lấy trung bình. Ý tưởng rất hợp lý cho tình huống rẽ nhánh: ở ngã ba, trung bình cộng của rẽ trái và rẽ phải là đâm thẳng vào dải phân cách.

Nhưng phải nói rõ phạm vi. Trong bản model này, quỹ đạo **không** dùng đa giả thuyết. Dòng `parse_mdn('plan', outs, in_N=0, ...)` ở `parse_model_outputs.py:113` truyền `in_N` bằng 0, tức tắt hẳn nhánh đó. Hằng số `PLAN_MHP_N = 5` vẫn nằm trong tệp constants nhưng không được dùng tới. Cơ chế này chỉ thực sự hoạt động cho đầu ra xe phía trước, với `LEAD_MHP_N = 2` giả thuyết và 3 mốc thời gian được chọn ra.

### 1.5. Lệnh lái là gia tốc ngang, không phải độ cong

```python
# selfdrive/modeld/modeld.py:54-55
desired_accel     = model_output['action'][0,1]
desired_curvature = model_output['action'][0,0] / (max(1.0, v_ego))**2
```

Đầu `action` xuất ra gia tốc ngang; độ cong chỉ là phép chia cho bình phương vận tốc.

Cách làm này hợp lý về mặt vật lý. Cùng một khúc cua, ở 20 km/h và 60 km/h cần độ cong như nhau nhưng cảm giác hoàn toàn khác. Cái mà hành khách cảm nhận và lốp phải chịu là gia tốc ngang, nên huấn luyện trên đại lượng đó thì mạng học được thứ bất biến theo tốc độ.

Kiểm chứng trên một khung hình thật: `desiredCurvature` = +0,00986 1/m ở vận tốc 6,42 m/s, tương ứng gia tốc ngang 0,406 m/s².

### 1.6. Model được cho biết trước độ trễ nó phải bù

```python
# selfdrive/modeld/modeld.py:297-300
frame_delay  = DT_MDL       # 50 ms, từ lúc chụp ảnh đến giờ
action_delay = DT_MDL / 2   # 25 ms, giữa hai lần chạy model
lat_action_t = lat_delay + frame_delay + action_delay
```

Biến `action_t` nằm trong danh sách đầu vào của mạng chính sách. Model không xuất ra lệnh cho thời điểm hiện tại rồi để ai đó bù trễ phía sau. Nó được cho biết lệnh sẽ có hiệu lực sau bao lâu và tự dự đoán cho đúng thời điểm đó.

Giá trị `lat_delay` trên xe này không phải hằng số cấu hình mà là đại lượng học trực tuyến, do tiến trình `lagd` ước lượng và công bố qua bản tin `liveDelay`:

| Thành phần | Giá trị đo trên log |
|---|---|
| `liveDelay.lateralDelay` (học được) | 0,326 s — trạng thái "estimated", hiệu chuẩn 100% |
| `carParams.steerActuatorDelay` (cấu hình) | 0,050 s |
| frame_delay + action_delay | 0,075 s |
| **→ lat_action_t thực tế** | **0,401 s** |
| → long_action_t (trễ ga-phanh 0,5 s + làm mượt 0,3 s) | 0,875 s |

Con số 0,4 giây đáng để dừng lại một chút: lệnh lái mà model xuất ra không dành cho hiện tại mà dành cho vị trí xe sẽ ở gần nửa giây nữa. Ở 60 km/h, đó là 6,7 mét phía trước.

Một chi tiết cấu hình nữa: `LAT_SMOOTH_SECONDS = 0.0` trong bản này, tức làm mượt lệnh lái đã bị tắt hẳn. Phần dọc vẫn làm mượt 0,3 giây, và độ trễ do làm mượt gây ra được cộng thẳng vào `long_delay` để model tự bù.

---

## Phase 2 — controlsd: từ độ cong đến vô-lăng

### 2.1. Xe này điều khiển theo góc, không theo mô-men

Phần lớn tài liệu openpilot nói về `latcontrol_torque`, tức tính mô-men xoắn đẩy vào vô-lăng. Xe này đi nhánh khác. Log xác nhận rõ: 5988 trên 5988 bản tin đều là `angleState`, và `actuators.torque` luôn bằng 0.

| | Điều khiển mô-men | Điều khiển góc (xe này) |
|---|---|---|
| Openpilot gửi xuống | lực xoắn tính bằng Nm | góc vô-lăng mong muốn tính bằng độ |
| Ai quay vô-lăng | Openpilot, trực tiếp | hệ thống lái của xe tự làm |
| Cần biết gì về xe | quan hệ mô-men và gia tốc ngang | tỉ số truyền lái |
| Xe điển hình | Honda, Toyota | Tesla, VinFast, nhiều xe điện |

### 2.2. Chuỗi biến đổi năm bước

**Bước 1 — lấy lệnh từ model**

```python
# selfdrive/controls/controlsd.py:192
new_desired_curvature = model_v2.action.desiredCurvature if CC.latActive else self.curvature
```

Khi không lái tự động, lệnh được gán bằng độ cong hiện tại, nên sai số bám lệnh luôn bằng không. Nhớ điều này, vì nó là lý do mọi chỉ số bám lệnh chỉ có nghĩa khi hệ thống đang hoạt động.

**Bước 2 — cộng thiên lệch sang trái, chỉ áp cho VinFast**

```python
# selfdrive/controls/controlsd.py:201-231
if self.CP.brand == "vinfast" and CC.latActive:
    if   v_ego_kph < 20.0: offset_max = -0.0015
    elif v_ego_kph < 40.0: offset_max = -0.001
    else:                  # giảm dần theo hàm mũ tới 60 km/h
    fade = math.exp(-abs(new_desired_curvature) / 0.002)
    new_desired_curvature += offset_max * fade
```

Đây là một bản vá thủ công đẩy xe lệch trái ở tốc độ thấp. Hệ số `fade` làm thiên lệch mờ đi khi xe đang vào cua, và toàn bộ cơ chế tắt hẳn trên 60 km/h.

Câu hỏi là thiên lệch này thực sự lớn đến đâu. Đo trên log, câu trả lời phụ thuộc hoàn toàn vào việc đường đang thẳng hay cong:

| Tình huống | Số frame | Thiên lệch áp dụng | So với lệnh của model |
|---|---|---|---|
| Gần như đi thẳng (\|k\| < 0,0005) | 36 | 0,00108 1/m | 430% |
| Đường cong nhẹ | 117 | 0,00058 1/m | 43% |
| Đang vào cua | 360 | 0,00020 1/m | 6% |
| Toàn bộ | 513 | 0,00026 1/m | 9% |

Nói cách khác: trên đường thẳng, lệnh lái gần như hoàn toàn là thiên lệch nhân tạo chứ không phải thứ model nhìn thấy. Khi vào cua thì nó gần như biến mất. Về mặt vật lý, 0,00108 1/m ở 22 km/h tương đương gia tốc ngang 0,040 m/s², đủ để đẩy xe lệch khoảng 0,18 mét sau 3 giây nếu không có gì bù lại.

**Bước 3 — cắt theo giới hạn an toàn ISO**

```python
# selfdrive/controls/lib/drive_helpers.py:26-40
max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)      # MAX_LATERAL_JERK = 5.0 m/s³
max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll * 9.81   # = 3.0 m/s²
new_curvature = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)   # 0.2
```

Cả hai giới hạn đều chia cho bình phương vận tốc, nên càng chạy nhanh càng bị siết chặt:

| Tốc độ | Độ cong tối đa | Bán kính cua nhỏ nhất |
|---|---|---|
| 22 km/h | 0,081 1/m | 12 m |
| 60 km/h | 0,011 1/m | 93 m |
| 100 km/h | 0,004 1/m | 260 m |

Ở 100 km/h, dù model có muốn bẻ gấp đến đâu, hệ thống cũng không cho phép vào cua gắt hơn bán kính 260 mét.

**Bước 4 — đổi độ cong thành góc vô-lăng**

```python
# opendbc/car/vehicle_model.py:104
return (curv - roll_compensation(roll, u)) * self.sR / self.curvature_factor(u)
```

Đây là mô hình xe đạp. Hệ số độ cong phụ thuộc tốc độ vì ở tốc độ cao lốp trượt ngang, cùng một góc vô-lăng cho độ cong nhỏ hơn. Ba tham số dùng ở đây đều là loại học trực tuyến chứ không lấy từ cấu hình:

| Tham số | Cấu hình gốc | Học được từ xe này |
|---|---|---|
| `steerRatio` — tỉ số truyền lái | 15,5 | 14,72 |
| `stiffnessFactor` — độ cứng lốp | 1,0 | 0,98 |
| `angleOffsetDeg` — lệch tâm vô-lăng | 0 | −1,09° |

**Bước 5 — bộ PI riêng của fork**

Ngoài góc cơ bản, fork còn chạy một bộ điều khiển PID riêng cho VinFast. Hằng số khác nhau rất xa giữa các dòng xe:

| Hằng số | VF6 (xe này) | VF8 | VF9 | Ý nghĩa |
|---|---|---|---|---|
| `VINFAST_ANGLE_KI` | 0,042 | 0,001 | 0,0006 | hệ số tích phân |
| `MAX_PI_CORR_RATE` | 0,75°/chu kỳ | 0,35° | 0,18° | tốc độ chỉnh tối đa |
| `VINFAST_MAX_ANGLE_CORR` | 22° | 15° | 10° | trần chỉnh |
| `SMALL_ANGLE_DEADBAND` | 0,18° | — | 0,4° | vùng chết bỏ qua sai số nhỏ |
| `VINFAST_ANGLE_KD` | 0,06 | 0 | 0,04 | hệ số vi phân |

VF6 có hệ số tích phân gấp 42 lần VF8 và trần chỉnh gấp đôi VF9. Khó đọc con số này theo cách nào khác ngoài: hệ thống lái của VF6 khó điều khiển hơn nhiều, và đội phát triển đã phải nới tay rất mạnh.

### 2.3. Đo thực tế: xe chỉ thực hiện 44% góc được lệnh

Tôi tự tính lại góc cơ bản từ `desiredCurvature` theo đúng công thức mô hình xe, rồi so với góc thật trong log, trên 2559 khung hình xe đang lái tự động:

| Đại lượng | Trung vị |
|---|---|
| Góc cơ bản tính từ độ cong | 4,93° |
| Phần bộ PI cộng thêm | 5,09° |
| Số lần chạm trần 22° | 0% |

Phần PI lớn ngang phần cơ bản. Câu hỏi tiếp theo là xe có thực hiện đúng lệnh không:

| Hồi quy | Hệ số | Tương quan |
|---|---|---|
| Vô-lăng thực = k × lệnh đầy đủ | 0,44 | 0,79 |
| Vô-lăng thực = k × góc cơ bản | 0,78 | 0,91 |

Xe chỉ thực hiện khoảng 44% góc được lệnh, nhưng lại bám khá sát góc cơ bản mà vật lý đòi hỏi.

Tôi đã thử xem đây có phải độ trễ không. Dịch lệnh đi từ 0,05 đến 0,40 giây thì sai số chỉ tăng đều từ 5,00° lên 5,61°, không hề giảm. Vậy không phải trễ, mà là thiếu hụt thường trực: hệ thống lái chỉ làm một phần việc được yêu cầu, bộ PI đẩy thêm để bù, và điểm cân bằng rơi đúng vào chỗ vô-lăng thực trùng với góc mà vật lý cần. Bảng hằng số ở bước 5 tồn tại chính vì lý do này.

Cần lưu ý: con số này đến từ một đoạn 25 giây ở khoảng 22 km/h. Đó là giả thuyết có bằng chứng, chưa phải kết luận cho mọi dải tốc độ.

---

## Phase 3 — longitudinal planner: bài toán tối ưu cho ga và phanh

Phần ngang đã giao gần hết cho mạng nơ-ron. Phần dọc thì ngược lại: mạng chỉ góp một phiếu, còn quyết định chính do một bài toán tối ưu giải lại từ đầu 20 lần mỗi giây.

Lý do nằm ở bản chất bài toán. Ga và phanh có ràng buộc cứng không được phép vi phạm — không được đâm vào xe trước — mà mạng nơ-ron thì không cho bảo đảm nào, còn bài toán tối ưu có ràng buộc thì có.

### 3.1. MPC hoạt động thế nào

MPC, điều khiển dự báo theo mô hình, làm việc như người chơi cờ tính trước vài nước: mô phỏng 10 giây tới với mọi cách đạp ga và phanh, chấm điểm từng phương án, chọn phương án tốt nhất, thực hiện đúng bước đầu tiên rồi vứt phần còn lại đi, và 50 mili-giây sau làm lại từ đầu.

Bước cuối là điều phản trực giác nhất: tính cả kế hoạch 10 giây nhưng chỉ dùng 0,05 giây đầu. Làm vậy vì kế hoạch xa chỉ để định hướng cho quyết định gần. Muốn biết bây giờ có nên nhả ga không thì phải biết 5 giây nữa sẽ ra sao.

### 3.2. Bài toán được viết ra thế nào

```python
# selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py:96-102
x_ego, v_ego, a_ego = SX.sym('x_ego'), SX.sym('v_ego'), SX.sym('a_ego')
model.x = vertcat(x_ego, v_ego, a_ego)      # trạng thái: vị trí, vận tốc, gia tốc
j_ego = SX.sym('j_ego')
model.u = vertcat(j_ego)                    # điều khiển: độ giật
f_expl = vertcat(v_ego, a_ego, j_ego)       # động học
```

Chi tiết tinh tế nhất của cả module nằm ở đây: biến điều khiển không phải gia tốc mà là độ giật, tức đạo hàm của gia tốc. Gia tốc bị đẩy xuống thành trạng thái.

Hệ quả là gia tốc không thể nhảy bậc, vì muốn đổi gia tốc thì phải tích phân độ giật qua thời gian. Tính êm ái được cài sẵn vào cấu trúc bài toán chứ không phải thêm bằng luật. Đây là lý do openpilot không bao giờ giật cục kiểu bật tắt.

Lưới thời gian dùng 12 điểm chia theo hàm bậc hai giống model, điểm cuối ở 10 giây. Chỉ 12 điểm cho 10 giây, vì giải nhanh quan trọng hơn chi tiết ở tương lai xa.

### 3.3. Hàm mục tiêu

| Thành phần | Trọng số | Ý nghĩa |
|---|---|---|
| Lệch khỏi khoảng cách mong muốn | 3 | giữ đúng cự ly với vật cản |
| Vị trí, vận tốc, gia tốc | 0 | tắt hoàn toàn |
| Đổi gia tốc so với chu kỳ trước | 200 | đừng đổi ý |
| Độ giật | 5 | đi cho êm |

Trọng số 200 cho việc đổi ý lớn hơn trọng số 3 cho việc giữ đúng khoảng cách tới 67 lần. Phạt nặng như vậy để lời giải không nhảy qua lại giữa hai phương án gần bằng điểm nhau — thứ sẽ biến thành rung giật mà hành khách cảm nhận rõ. Hệ thống thà bám sai khoảng cách một chút còn hơn đổi ý liên tục.

Thành phần đầu tiên còn được chia cho `(v_ego + 10)`. Sai lệch 5 mét khi đứng yên là nghiêm trọng, nhưng 5 mét khi chạy 100 km/h thì không đáng kể. Phép chia này chuẩn hoá sai số khoảng cách thành sai số thời gian, để một bộ trọng số dùng được ở mọi tốc độ.

Hai trọng số cuối còn được nhân với hệ số theo chế độ lái: chế độ aggressive dùng hệ số 0,5, tức chấp nhận giật gấp đôi.

### 3.4. Mẹo hay nhất: mọi thứ đều là một vật cản

MPC chỉ biết một khái niệm duy nhất — có một vật cản đứng yên ở toạ độ `x_obstacle`. Ba tình huống hoàn toàn khác nhau đều được quy về đúng con số đó.

```python
# long_mpc.py:86-87   xe phía trước đang chạy
def get_stopped_equivalence_factor(v_lead):
    return (v_lead**2) / (2 * COMFORT_BRAKE)      # COMFORT_BRAKE = 2.5 m/s²

# long_mpc.py:353-354   chọn cái gần nhất
x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]
```

Xe trước đang chạy 40 km/h thì nếu phanh gấp nó vẫn trôi thêm 24,7 mét, nên coi như có một bức tường đứng yên ở vị trí xe đó cộng thêm 24,7 mét. Bài toán đuổi theo vật chuyển động biến thành giữ khoảng cách với vật đứng yên. Tốc độ cài đặt cruise cũng được biến thành một vật cản ảo trôi đi đúng bằng tốc độ đó.

Toàn bộ logic khi nào bám xe, khi nào chạy theo tốc độ cài gói gọn trong một lệnh `argmin`. Không máy trạng thái, không if-else. Đây là chỗ đáng học nhất trong module.

Đo trên log, nguồn quyết định phân bố như sau: cruise 55%, e2e 36%, lead0 8%, lead1 1%.

### 3.5. Khoảng cách mong muốn

```python
# long_mpc.py:89-90
def get_safe_obstacle_distance(v_ego, t_follow):
    return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE
```

| Số hạng | Ý nghĩa |
|---|---|
| v² / (2 × 2,5) | quãng đường tự phanh êm về 0 |
| t_follow × v | khoảng cách đi được trong thời gian phản ứng |
| + 6,0 m | khoảng hở khi cả hai đã dừng hẳn |

Hệ số `t_follow` phụ thuộc chế độ lái: aggressive 1,25 giây, standard 1,45 giây, relaxed 1,75 giây. Xe này đang để relaxed.

| Tốc độ | Tổng | Phanh | 1,75 s | Dừng |
|---|---|---|---|---|
| 10 km/h | 12,4 m | 1,5 | 4,9 | 6,0 |
| 20 km/h | 21,9 m | 6,2 | 9,7 | 6,0 |
| 30 km/h | 34,5 m | 13,9 | 14,6 | 6,0 |
| 40 km/h | 50,1 m | 24,7 | 19,4 | 6,0 |
| 60 km/h | 90,7 m | 55,6 | 29,2 | 6,0 |

Số hạng phanh tăng theo bình phương tốc độ nên ở 60 km/h nó chiếm 61% tổng. Nếu thấy xe giữ khoảng cách xa quá mức trên đường thoáng thì đây là nguyên nhân, và chuyển sang chế độ standard sẽ rút ngắn đáng kể.

Kiểm chứng trên 487 khung hình xe đang giữ ga và có xe phía trước, tốc độ trung vị 22 km/h: khoảng cách thực tế 28,9 mét so với 24,4 mét mà MPC mong muốn, tỉ lệ 1,19. Xe giữ xa hơn mức tối thiểu, hợp lý vì phần lớn thời gian nó bị giới hạn bởi tốc độ cài đặt chứ không phải bởi xe phía trước.

### 3.6. Ràng buộc mềm và bộ giải

Tất cả ràng buộc đều là loại mềm: được phép vi phạm nhưng trả giá rất đắt.

Lý do rất thực dụng. Bài toán với ràng buộc cứng có thể không có lời giải, chẳng hạn khi xe trước phanh gấp thì mọi phương án đều vi phạm. Ràng buộc cứng thì bộ giải trả về lỗi và hệ thống đứng hình; ràng buộc mềm thì luôn có lời giải, và lời giải đó là vi phạm ít nhất có thể. Trong an toàn, một câu trả lời tệ vẫn tốt hơn không có câu trả lời nào.

Ràng buộc vùng nguy hiểm dùng hệ số `LEAD_DANGER_FACTOR = 0,75`, tức vùng cấm chỉ tính từ 75% khoảng cách mong muốn trở vào, tạo một dải đệm.

Về bộ giải, cấu hình là SQP_RTI: mỗi chu kỳ chỉ chạy đúng một vòng lặp Newton, lấy lời giải chu kỳ trước làm điểm xuất phát. Vì bài toán chỉ đổi chút ít sau mỗi 50 mili-giây nên một vòng lặp là đủ gần. Comment trong mã nguồn nói thẳng triết lý: bộ giải HPIPM cho lời giải dùng được kể cả khi bị dừng sớm, điều đó rất quan trọng khi thời gian tính toán bị chặn cứng.

Đo trên log: thời gian giải trung vị 0,59 mili-giây, phân vị 99 là 10,28 mili-giây, trên ngân sách 50 mili-giây. Rất dư dả.

### 3.7. Mạng nơ-ron chen vào ở đâu

```python
# selfdrive/controls/lib/longitudinal_planner.py:328-330
if self.is_e2e(sm):
    output_a_target = min(output_a_target_e2e, output_a_target_mpc)
```

Phép lấy giá trị nhỏ hơn này là cả một tuyên bố về triết lý thiết kế: model chỉ được phép làm xe chậm lại, không bao giờ được phép làm xe nhanh hơn mức MPC cho phép.

Model nhìn thấy những thứ MPC mù tịt — đèn đỏ, người băng qua đường, xe rẽ cắt đầu. Nhưng model cũng có thể nhầm. Cho nó quyền phủ quyết theo một chiều là cách lấy được cái lợi mà chặn được cái hại: model nhầm thì xe chỉ chạy chậm hơn cần thiết, chứ không bao giờ vì model nhầm mà đâm.

Con số 36% trong log nghĩa là hơn một phần ba thời gian, model phanh sớm hơn MPC và model thắng. Trên toàn route, cách đếm hơi khác — theo nhãn nguồn quyết định mà planner tự ghi lại — cho 52% (mục 9.3). Hai con số đo hai thứ không hoàn toàn giống nhau, nhưng cùng chỉ về một điều: nhánh mạng nơ-ron là nguồn phanh chủ đạo chứ không phải trường hợp ngoại lệ.

### 3.8. Bốn lớp giới hạn cuối

| Lớp | Quy tắc |
|---|---|
| Trần gia tốc theo tốc độ | 1,6 / 1,2 / 0,8 / 0,6 m/s² tại 0 / 10 / 25 / 40 m/s |
| Giảm ga khi vào cua | tổng gia tốc dọc và ngang không vượt ngưỡng lốp |
| Thả trôi | nếu `throttle_prob` < 0,4 thì chặn ga, chỉ cho trôi theo quán tính |
| Giới hạn tuyệt đối | gia tốc từ −3,5 đến +2,0 m/s² |

Bản thân các ngưỡng cắt này cũng bị giới hạn tốc độ thay đổi ở mức 0,05 m/s² mỗi chu kỳ, để khi điều kiện đổi đột ngột thì trần không nhảy bậc kéo theo lệnh nhảy bậc.

---

## Phase 4 — radard: ghép radar với camera

Xe có hai con mắt nhìn về phía trước và chúng giỏi những thứ khác nhau.

| | Radar | Camera |
|---|---|---|
| Khoảng cách | rất chính xác | kém dần theo khoảng cách |
| Vận tốc tương đối | đo trực tiếp bằng hiệu ứng Doppler | phải suy từ đạo hàm |
| Biết đó là cái gì | không biết gì cả | biết là xe, người hay biển báo |
| Vật đứng yên | lẫn với biển báo, nắp cống | phân biệt được |
| Mưa, tối, chói nắng | không ảnh hưởng | kém đi |

### 4.1. Radar trên xe này cho gì

Đây không phải radar cho đám mây điểm thô mà là danh sách đối tượng do radar của chính xe xuất ra. Đo trên log:

| Chỉ số | Giá trị |
|---|---|
| Số đối tượng mỗi bản tin | 0 điểm 28%, 1 điểm 59%, 2 điểm 13% |
| Tổng số điểm trong segment | 1337 |
| Cờ `measured` (đo mới, không ngoại suy) | 0% — luôn bằng False |
| Có vận tốc tương đối khác 0 | 72% |
| Khoảng cách trung vị | 26,1 m |

Radar chỉ theo được nhiều nhất hai đối tượng, và hơn một phần tư số điểm không có vận tốc. Điều này giải thích vì sao fork phải có cả một nhóm hằng số `POSITION_ONLY` dành riêng cho các vệt radar chỉ có vị trí mà không có vận tốc tin cậy.

### 4.2. Theo dõi từng đối tượng bằng Kalman filter

Một lưu ý về xuất xứ mã nguồn trước khi đi tiếp. Trong bản fork, tệp `selfdrive/controls/radard.py` chỉ là một lớp vỏ 17 dòng nạp bytecode từ `__pycache__/radard_impl.pyc`; mã thật không đọc trực tiếp được. Vì vậy các đoạn trích dưới đây lấy từ bản gốc của comma, nơi cùng những hàm đó tồn tại dưới dạng mã nguồn. Tôi đã dịch ngược tệp `.pyc` của fork để xác nhận nó có đúng các hàm này — `get_RadarState`, `match_vision_to_track`, `laplacian_pdf`, `get_lead` — và phần khác biệt của fork nằm ở hơn 150 hằng số bổ sung, sẽ nói ở mục 4.5.

```python
# bản gốc comma — selfdrive/controls/radard.py:35-37
self.A = [[1.0, dt], [0.0, 1.0]]     # trạng thái: [vận tốc, gia tốc]
self.C = [1.0, 0.0]                  # chỉ đo được vận tốc
```

Radar cho khoảng cách và vận tốc tương đối nhưng không cho gia tốc, mà MPC ở Phase 3 lại cần gia tốc của xe trước để dự báo. Nên mỗi đối tượng được gắn một bộ lọc Kalman, trong đó gia tốc là đại lượng ẩn phải suy ra từ chuỗi vận tốc theo thời gian.

Một chi tiết thực dụng: hệ số Kalman không được tính lúc chạy mà tra từ bảng cứng 20 giá trị dựng sẵn. Comment giải thích lý do — tính hệ số từ ma trận cần thư viện điều khiển, mà cài thêm thư viện lên thiết bị nhúng thì không đáng. Với hệ tuyến tính bất biến, hệ số hội tụ về giá trị cố định nên tra bảng cho kết quả y hệt.

```python
# bản gốc comma — selfdrive/controls/radard.py:75-78
if abs(self.aLeadK) < 0.5:
    self.aLeadTau.x = _LEAD_ACCEL_TAU      # = 1.5 s
else:
    self.aLeadTau.update(0.0)
```

Hằng số `aLeadTau` quyết định mức độ tin vào gia tốc của xe trước khi ngoại suy. Nếu xe trước gia tốc nhẹ thì có thể chỉ là nhiễu, giả định nó sẽ hết nhanh. Nếu gia tốc mạnh thì đó là hành động có chủ ý, tau giảm về 0 nghĩa là tin rằng gia tốc sẽ duy trì. Đúng trực giác: xe trước đạp phanh gấp thì sẽ phanh tiếp chứ không nhả ra ngay.

### 4.3. Ghép radar với xe mà camera nhìn thấy

```python
# bản gốc comma — selfdrive/controls/radard.py:106-118
def laplacian_pdf(x, mu, b):
    return math.exp(-abs(x-mu)/b)

def prob(c):
    prob_d = laplacian_pdf(c.dRel,         offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel,         -lead.y[0],         lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0],          lead.vStd[0])
    return prob_d * prob_y * prob_v
```

Với mỗi đối tượng radar, tính xác suất nó chính là chiếc xe camera đang nhìn, dựa trên ba chiều: khoảng cách, lệch ngang, vận tốc. Nhân ba xác suất rồi chọn điểm cao nhất.

Điểm tinh tế nhất của cả module nằm ở chỗ bề rộng phân phối chính là `xStd`, `yStd` và `vStd` mà model tự khai báo. Đây là nơi cái ý tưởng "model xuất ra phân phối chứ không xuất ra con số" ở Phase 1 được dùng thật sự. Khi camera không chắc về khoảng cách thì `xStd` lớn, phân phối rộng ra, và tiêu chí khoảng cách tự động mất trọng lượng — việc ghép sẽ dựa nhiều hơn vào lệch ngang và vận tốc. Hệ thống tự điều chỉnh mức tin cậy mà không cần một dòng `if` nào.

Chọn được điểm tốt nhất vẫn chưa đủ, nó còn phải qua cổng kiểm tra tỉnh táo: sai lệch khoảng cách phải nhỏ hơn 25% khoảng cách hoặc 5 mét, tuỳ cái nào lớn hơn. Không qua được thì hệ thống dùng thị giác thuần.

### 4.4. Thứ tự ưu tiên khi chọn xe phía trước

| Mức | Khi nào | Cờ radar |
|---|---|---|
| 1. Radar và thị giác khớp nhau | tốt nhất | True |
| 2. Chỉ thị giác | không ghép được đối tượng radar nào | False |
| 3. Chỉ radar | camera yếu nhưng vật nằm trong đường đi dự kiến | True |
| 4. Ghi đè tốc độ thấp | vật rất gần, xe đang bò chậm | True |

Mức 4 là cơ chế chống đâm khi kẹt xe: vật cách từ 0,75 đến 25 mét, lệch ngang dưới 1 mét, xe chạy dưới 4 m/s. Camera có thể không nhận ra vật chắn phía trước, nhưng nếu radar thấy có gì rất gần thì cứ phanh.

### 4.5. Fork viết lại gần như toàn bộ module này

Bản gốc có khoảng 10 hằng số. Bản fork có hơn 150. Đọc tên chúng là thấy ngay đang chống chọi với cái gì:

| Nhóm hằng số | Vấn đề đang giải |
|---|---|
| `HARD_PED_REJECT`, `SLOW_VRU_REJECT` | người đi bộ và xe máy sát xe, không được coi là xe dẫn đường |
| `PARKED_ROADSIDE` | xe đỗ bên lề, đừng phanh vì chúng |
| `ONCOMING`, `URBAN_CROSSING` | xe ngược chiều và xe cắt ngang ở ngã tư |
| `LATERAL_FLYBY`, `SIDE_PASS`, `OVERTAKE_PASS` | xe lướt qua bên hông, đừng bám nhầm |
| `URBAN_CREEP`, `LOW_SPEED`, `CREEP_LEAD` | bò trong tắc đường |
| `POSITION_ONLY` | vệt radar chỉ có vị trí, không có vận tốc tin cậy |
| `LEAD_STABILITY`, `LEAD_HYSTERESIS` | đừng đổi mục tiêu xoành xoạch |

Một ví dụ cụ thể: khi xe bò dưới 10 km/h và có vật di chuyển chậm hơn 11 km/h nằm lệch ngang trong 1,2 mét, nó bị loại thẳng khỏi danh sách ứng viên. Rõ ràng để xe không đứng chôn chân vì xe máy lách qua đầu.

Đây là bằng chứng rõ nhất trong cả dự án rằng openpilot gốc được thiết kế cho đường cao tốc Mỹ, và phần lớn công sức của đội phát triển đổ vào việc làm nó sống được trong giao thông Việt Nam.

### 4.6. Đo thực tế: thị giác báo khoảng cách hụt bao nhiêu

Đo trên toàn bộ 52 segment, 21.940 khung hình có đồng thời cả radar lẫn thị giác nhìn thấy xe phía trước.

Trước hết phải xác định hệ quy chiếu. Nhánh thị giác thuần trừ đi hằng số `RADAR_TO_CAMERA = 1,52` mét vì trong thiết kế của comma, radar lắp trước camera. Câu hỏi là hằng số đó có đúng cho VinFast không, và dữ liệu trả lời là không:

| Cự ly theo radar | Số frame | Lệch thô (x[0] − dRel) |
|---|---|---|
| 0–8 m | 7.437 | −1,50 m |
| 8–12 m | 4.948 | −2,73 m |
| 12–16 m | 2.003 | −1,89 m |
| 16–22 m | 2.540 | −1,87 m |
| 22–30 m | 2.443 | −0,73 m |
| 30–45 m | 1.742 | −1,02 m |
| Trên 45 m | 827 | +0,11 m |

Nếu radar thật sự đo từ một điểm cách camera 1,52 mét thì cột cuối phải là hằng số +1,52 ở mọi cự ly. Thực tế nó tiến về 0 khi ra xa. Kết luận: `dRel` của radar VinFast đã nằm cùng hệ quy chiếu với camera, nên với xe này không được trừ 1,52 mét. Bộ phân tích tín hiệu radar của VinFast là mã đóng nên không kiểm chứng được bằng cách đọc mã nguồn; đây là suy ra từ số liệu.

Với cách so sánh đúng đó, chất lượng ước lượng như sau:

| Chỉ số | Giá trị | Nhận xét |
|---|---|---|
| Tương quan thị giác vs radar | 0,967 | rất cao |
| Hệ số tỉ lệ | 1,034 | thang đo đúng, không co giãn |
| Lệch hệ thống | −1,69 m | thị giác báo gần hơn |
| Độ tán mạc quanh mức lệch | 1,10 m | độ lệch tuyệt đối trung vị sau khi trừ mức lệch — đây mới là sai số ngẫu nhiên |
| Sai số tuyệt đối median / P90 | 2,20 m / 3,87 m | |
| xStd model tự báo | 0,95 m | tỉ lệ sai số trên xStd = 1,89 |

Mức lệch phụ thuộc cự ly theo chiều ngược với trực giác: dưới 12 mét thị giác báo hụt khoảng 2 mét, tức 27%; trên 30 mét thì hai nguồn khớp nhau trong vòng 0,7 mét. Giải thích hợp lý nhất là ở cự ly gần, xe phía trước choán gần hết khung hình và model ước lượng tới một điểm tham chiếu khác với điểm radar phản xạ.

Một điểm sáng: vận tốc tương đối thì thị giác đo rất chuẩn, lệch trung vị chỉ +0,01 m/s với độ tán mạc 0,26 m/s. Model nhìn sai khoảng cách nhưng nhìn đúng tốc độ tiếp cận, và với việc phanh thì tốc độ tiếp cận mới là đại lượng quyết định.

Cần nói rõ: kết quả này khác hẳn khi chỉ đo trên một segment. Bản đầu của tài liệu dùng segment_22 và cho ra bức tranh ngược lại — chính xác ở gần, hụt 12–14% ở xa. Đó là lý do phải chạy toàn route.

### 4.7. Vì sao nhiều khung hình không có radar hậu thuẫn

| Điều kiện | Số frame | Tỉ lệ có radar |
|---|---|---|
| Xe chạy 11–25 km/h | 591 | 14% |
| Xe chạy trên 25 km/h | 506 | 67% |
| Vật cách 10–20 m | 208 | 29% |
| Vật cách 20–30 m | 695 | 47% |
| Vật cách trên 30 m | 194 | 17% |

Yếu tố quyết định là tốc độ chứ không phải khoảng cách. Chạy nhanh thì radar hoạt động tốt; bò trong phố thì gần như chỉ còn camera — hợp lý với các bộ lọc loại người đi bộ và xe máy ở trên.

Mục tiêu radar cũng không bền: đổi số hiệu 11 lần trong 420 khung hình, dùng qua 10 số hiệu khác nhau. Đó là lý do fork phải có cơ chế giữ mục tiêu và trễ chuyển đổi.

Trên toàn bộ 52 segment, tỉ lệ phủ sóng radar cao hơn nhiều so với riêng segment_22: trung vị 54% số khung hình có xe phía trước được radar hậu thuẫn, và xe có xe phía trước trong 72% thời gian.

---

## Phase 5 — selfdrived và panda: ai quyết định, ai phủ quyết

Bốn Phase trước nói về việc tính ra lệnh gì. Phase này nói về việc có được phép ra lệnh hay không. Hai lớp hoàn toàn tách biệt, nằm ở hai nơi khác nhau, viết bằng hai ngôn ngữ khác nhau.

### 5.1. Máy trạng thái điều phối

| Trạng thái | Nghĩa | Xe có được lái không |
|---|---|---|
| `disabled` | tắt | không |
| `preEnabled` | đã bấm bật nhưng còn điều kiện chờ | chưa |
| `enabled` | đang chạy bình thường | có |
| `overriding` | người đang can thiệp, hệ thống nhường | một phần |
| `softDisabling` | đang đếm ngược 3 giây để tắt | vẫn có |

Trạng thái `softDisabling` là thiết kế đáng chú ý nhất. Khi gặp sự cố không nguy hiểm tức thì, hệ thống không tắt ngay mà báo động và đếm ngược 3 giây; nếu trong khoảng đó điều kiện trở lại bình thường thì quay về trạng thái chạy như chưa có gì. Lý do: tắt đột ngột giữa khúc cua nguy hiểm hơn là gắng gượng thêm 3 giây trong khi tài xế kịp nắm vô-lăng.

### 5.2. Hệ thống sự kiện

Máy trạng thái không biết gì về camera, radar hay vô-lăng. Nó chỉ biết các loại sự kiện. Module nào phát hiện vấn đề thì đẩy tên sự kiện vào danh sách, máy trạng thái đọc danh sách và quyết định. Không module nào được phép tự tắt hệ thống — quyền tắt tập trung ở một chỗ duy nhất.

| Loại sự kiện | Hậu quả | Ví dụ |
|---|---|---|
| ENABLE | cho phép bật | bấm nút resume |
| NO_ENTRY | chặn không cho bật | đang đạp phanh |
| OVERRIDE_LATERAL / LONGITUDINAL | nhường quyền tạm thời | người xoay vô-lăng, đạp ga |
| WARNING | chỉ cảnh báo, không đổi trạng thái | sắp chệch làn |
| SOFT_DISABLE | đếm ngược 3 giây rồi tắt | lỗi tạm thời |
| IMMEDIATE_DISABLE | tắt ngay lập tức | mất kết nối CAN |
| USER_DISABLE | người chủ động tắt | đạp phanh |

Bốn sự kiện có thật trong log:

| Sự kiện | Số lần | Cờ mang theo | Hậu quả thực tế |
|---|---|---|---|
| `cruiseMismatch` | 28 | không có cờ nào | không gì cả |
| `pedalPressed` | 10 | noEntry, userDisable | tắt ngay và chặn bật lại |
| `steerOverride` | 7 | overrideLateral | chuyển sang overriding |
| `pcmDisable` | 1 | userDisable | tắt |

Dòng đầu là một chi tiết đáng nhớ. Tên sự kiện nghĩa là openpilot muốn huỷ cruise của xe mà huỷ không được, nghe rất đáng lo, và nó xảy ra 28 lần. Nhưng nhìn vào định nghĩa trong `events.py` dòng 280 thì dòng duy nhất bên trong đã bị chú thích tắt, nên sự kiện được ghi vào log mà không mang cờ nào và máy trạng thái bỏ qua hoàn toàn.

Bài học đọc log: một sự kiện xuất hiện nhiều không có nghĩa nó gây hậu quả gì.

### 5.3. Panda — quyền phủ quyết bằng phần cứng

Panda là bo mạch cắm giữa thiết bị và xe, chạy firmware C, kiểm tra lại từng bản tin CAN trước khi cho đi qua. Triết lý: giả định phần mềm Python bên trên có thể sai bất cứ lúc nào, nên một lớp C nhỏ và kiểm chứng được sẽ chặn hậu quả.

Với điều khiển theo góc, nó kiểm tra bốn thứ:

- Giới hạn tốc độ đổi góc theo tốc độ xe — bản sao độc lập của `clip_curvature` ở Phase 2, cố tình làm hai lần ở hai nơi.
- Cố tình nới lỏng một chút, vì panda và openpilot đọc tốc độ ở hai thời điểm hơi lệch nhau nên giới hạn phải rộng hơn để không báo vi phạm oan.
- Giới hạn số bản tin trên mỗi khoảng thời gian, chống việc lách giới hạn bằng cách gửi dày hơn.
- Kiểm tra sai lệch giữa góc lệnh và góc đo được, không cho lệnh quá xa vị trí vô-lăng thật.

Log cho thấy cờ `controlsAllowed` bật 100% thời gian, không có lần chặn nào.

### 5.4. Điểm cần lưu ý về lớp an toàn của VinFast

```c
// opendbc/safety/modes/vinfast_stub.h
// VinFast enforcement lives in flashed Panda firmware
static safety_config vinfast_stub_init(uint16_t param) {
  controls_allowed = true;
}
static bool vinfast_stub_tx_hook(const CANPacket_t *msg) {
  return true;                       // cho mọi bản tin đi qua
}
```

Trong kho mã công khai, lớp an toàn của VinFast chỉ là một cái vỏ rỗng cho mọi thứ đi qua. Phần kiểm tra thật nằm trong firmware panda đã nạp sẵn, build từ các header riêng không công bố.

Điều này không có nghĩa xe không an toàn — firmware thật trên panda có thể đầy đủ kiểm tra. Nhưng nó có nghĩa là không thể kiểm chứng được từ kho mã này. Với openpilot của comma, ai cũng đọc được đúng những giới hạn mà xe đang chịu; với bản này thì phải tin lời đội phát triển.

Nếu chỉ hỏi được một câu về an toàn, câu đáng hỏi nhất là firmware panda đang chạy thực thi những giới hạn nào và có cách nào đọc chúng ra không.

---

## 6. Tổng hợp các bản vá riêng cho VinFast

Gom lại từ cả năm Phase, đây là những chỗ fork khác với openpilot gốc. Cột cuối cho biết bản vá đó có áp dụng cho chiếc VF6 này hay không.

| Bản vá | Nội dung | Áp cho VF6? |
|---|---|---|
| Thiên lệch lái sang trái | cộng thêm độ cong âm ở dưới 60 km/h, mờ dần khi vào cua | Có |
| Bộ PI điều khiển góc riêng | hệ số tích phân 0,042 — gấp 42 lần VF8; trần chỉnh 22° | Có |
| radard viết lại | hơn 150 hằng số cho giao thông đô thị Việt Nam | Có |
| Làm nhẹ phanh nhẹ | `VF_MILD_DECEL_SCALE = 0,93` | Không |
| Nhận diện đèn đỏ và bù phanh | `VF_REDLIGHT_ACCEL_SCALE = 1,03` | Không |
| Thu hẹp khoảng hở khi dừng | `VF_STOP_LEAD_GAP_M` theo chế độ lái | Không |
| Tăng nhẹ lệnh ga-phanh | hệ số 1,05 khi phanh và 1,08 khi ga | Không (chỉ VF9) |
| Nhích theo radar làn bên | `RadarPathNudge` | Không (VF8/VF9) |

Điểm đáng chú ý: toàn bộ nhóm bản vá về phanh đều bị chặn bởi một dòng điều kiện kiểm tra fingerprint có thuộc tập VF8 và VF9 hay không. Chiếc VF6 này chạy phần điều khiển dọc hoàn toàn gốc.

Việc đội phát triển phải viết riêng cả một tệp kiểm thử mang tên `test_vf_late_brake.py` cho thấy họ đã gặp vấn đề phanh muộn trên VF8 và VF9. Câu hỏi còn bỏ ngỏ là VF6 có cùng triệu chứng hay không.

---

## 7. Bảng số liệu đo được trên xe

Cột cuối cho biết con số đến từ đâu: một segment 60 giây, hay cả 52 segment của route.

| Đại lượng | Giá trị | Phạm vi |
|---|---|---|
| Thời gian chạy mạng nơ-ron | 25,8 ms (P90: 26,3) | 1 segment |
| Thời gian giải bài toán MPC | 0,59 ms (P99: 10,3 ms) | 1 segment |
| Độ trễ lái học được | 0,326 s | 1 segment |
| Tầm nhìn của lệnh lái | 0,401 s | 1 segment |
| Tỉ số truyền lái học được | 14,72 | 1 segment |
| Lệch tâm vô-lăng học được | −1,09° | 1 segment |
| Góc cơ bản từ độ cong | 4,93° | 1 segment |
| Phần PI cộng thêm | 5,09° | 1 segment |
| Tỉ lệ thực hiện góc lệnh | 44% | 1 segment |
| Thiên lệch trái trên đường thẳng | 0,00108 1/m | 1 segment |
| Khoảng cách thời gian trung vị | 3,40 s | cả route |
| Nguồn quyết định ga-phanh | cruise 35%, e2e 52%, lead 13% | cả route, lúc giữ ga |
| Lệch ước lượng khoảng cách | −1,69 m | cả route |
| Tỉ lệ khung hình có radar | 54% | cả route |
| Hệ số sai của aEgo trên CAN | 3,83 lần | 50 segment |
| Quãng đường dừng từ 18 km/h | 20,0 m (người: 7,4 m) | cả route |
| Độ giật khi giữ ga | 0,162 m/s³ (người: 0,227) | cả route |
| Can thiệp bằng chân phanh | 0,6 lần/km | cả route |
| Giành lại vô-lăng | 4,7 lần/km | cả route |
| Sai số dự đoán quỹ đạo 2 s | 0,245 m (người lái) | cả route |
| Hiệu chuẩn yStd ở mốc 2 s | 0,84 | cả route |
| Đánh vòng trên đường thẳng | 0,014 m (người: 0,019) | cả route |

---

## 8. Kết quả đo phần lái trên toàn route

Năm Phase ở trên giải thích hệ thống hoạt động thế nào. Hai mục 8 và 9 trả lời câu hỏi khác: nó hoạt động tốt đến đâu trên chính chiếc xe này. Mục 8 nói về phần lái, mục 9 nói về ga-phanh.

Phép đo dựa trên một nguyên tắc duy nhất: đáp án phải đến từ nguồn độc lập với thứ đang bị chấm điểm. Quỹ đạo thật của xe được dựng lại từ tốc độ bánh xe và con quay hồi chuyển, không dùng một byte nào từ camera hay từ chính model. Nhờ vậy khi so quỹ đạo model dự đoán với quỹ đạo xe đã đi, hai vế không hề chia sẻ sai số chung.

Bốn mươi lăm trên năm mươi hai segment vượt được vòng kiểm định: 45 phút, 15,3 km, trong đó Openpilot cầm lái 13,5 km, chiếm 68% thời gian và 88% quãng đường. Phần chênh giữa hai tỉ lệ là những lúc dừng đèn đỏ và bò trong tắc đường, khi tài xế lái. Tốc độ trung vị chỉ 22 km/h, phân vị 90 là 59 km/h. Đây là dữ liệu nội đô đông đúc, không phải cao tốc.

| Lý do loại segment | Số segment | Giải thích |
|---|---|---|
| IMU không khớp góc lái | 4 | tương quan giữa vận tốc góc đo được và góc vô-lăng dưới 0,91 — không dựng lại được quỹ đạo thật đáng tin |
| Xe gần như đứng yên | 3 | quãng đường dưới 10 mét, không có gì để đo |

Đầu mỗi lần phân tích có một phép thử dấu chạy tự động: tương quan giữa vị trí ngang model dự đoán ở mốc 2 giây và vị trí ngang xe thật sự tới, tính trên toàn bộ dữ liệu, bằng **+0,937**. Nếu quy ước dấu của hệ tọa độ bị hiểu ngược thì con số này sẽ âm. Chính bài kiểm tra này đã phát hiện một lỗi đảo dấu trong bản phân tích đầu tiên; xem thêm mục 10.4.

### 8.1. Model dự đoán quỹ đạo chính xác đến mức nào

Chỉ những đoạn tài xế cầm lái mới cho phép đo sạch. Khi Openpilot lái, xe đi theo đúng dự đoán của chính nó nên sai số nhỏ giả tạo; con số đó được ghi lại để tham khảo nhưng không dùng để đánh giá.

| Tầm dự đoán | Sai số trung vị | Phân vị 90 | Tỉ lệ sai quá 0,5 m | yStd model tự báo |
|---|---|---|---|---|
| 1 giây | 0,088 m | 0,551 m | 12,5% | 0,102 m |
| 2 giây | 0,245 m | 1,384 m | 33,8% | 0,288 m |
| 3 giây | 0,481 m | 3,012 m | 49,2% | 0,597 m |

Đọc theo quãng đường thay vì theo thời gian: sai 0,267 m ở mốc 10 mét phía trước, 0,539 m ở mốc 20 mét, 0,520 m ở mốc 30 mét.

Tính theo thời gian thì sai số tăng gần gấp đôi mỗi giây, nhanh hơn tuyến tính, vì sai lệch về hướng tích lũy dần thành sai lệch về vị trí.

Để so sánh, khi Openpilot cầm lái các con số lần lượt là 0,031 / 0,101 / 0,182 m. Chênh lệch gấp ba đến năm lần này không có nghĩa là model chính xác hơn khi nó tự lái; nó chỉ phản ánh việc bài toán trở thành vòng kín.

### 8.2. yStd — thước đo tự tin của model dùng được

Model không chỉ xuất ra quỹ đạo mà còn xuất ra độ lệch chuẩn cho từng điểm (mục 1.4). Một thước đo tin cậy tốt phải thỏa hai điều: đúng độ lớn, và xếp hạng đúng thứ tự.

| Tầm dự đoán | Sai số thật / yStd | Tương quan hạng | Kết luận |
|---|---|---|---|
| 1 giây | 0,92 | +0,77 | gần như hoàn hảo |
| 2 giây | 0,84 | +0,71 | hơi thận trọng |
| 3 giây | 0,77 | +0,74 | thận trọng hơn nữa |

Tỉ lệ dưới 1 nghĩa là model báo sai số lớn hơn sai số thật: nó khiêm tốn chứ không tự tin thái quá, và càng nhìn xa càng khiêm tốn. Tương quan hạng 0,71–0,77 cho biết những khung hình model tự nhận là khó thì đúng là khó thật.

Kết luận thực dụng: yStd có thể dùng trực tiếp làm tín hiệu cảnh báo. Khi nó vọt lên, hệ thống sắp gặp khó. Mục 8.4 cho thấy tín hiệu này thật sự có giá trị dự báo.

### 8.3. Giữ làn: ổn định hơn người, nhưng tay lái bận rộn hơn nhiều

Phép đo này cũng không dùng vạch kẻ đường. Mỗi cửa sổ 5 giây được đánh giá bằng biên độ đánh vòng — độ lệch của quỹ đạo thật so với một đường cong bậc ba khớp qua chính nó, tức là đã trừ đi độ cong của con đường.

| Nhóm | Số cửa sổ | Đánh vòng (m) | Đổi chiều vô-lăng (lần/phút) | Gia tốc ngang (m/s²) |
|---|---|---|---|---|
| Đường thẳng — Openpilot | 1.033 | 0,014 | 96 | 0,146 |
| Đường thẳng — người lái | 56 | 0,019 | 48 | 0,165 |
| Đường cong — Openpilot | 209 | 0,038 | 144 | 0,190 |
| Đường cong — người lái | 65 | 0,173 | 36 | 0,394 |

Hai kết luận trái chiều nhau nằm trong cùng một bảng.

Về vị trí, Openpilot giữ ổn định hơn người ở mọi điều kiện, và trên đường cong thì hơn hẳn: 3,8 cm so với 17,3 cm. Về cách cầm lái, nó đổi chiều vô-lăng gấp hai đến bốn lần người — 96 so với 48 lần mỗi phút trên đường thẳng, 144 so với 36 trên đường cong. Đó là dấu hiệu của một vòng điều khiển liên tục chỉnh sửa những sai lệch rất nhỏ: chính xác, nhưng người ngồi trong xe cảm nhận được sự bận rộn của vô-lăng.

Điều gì làm xe đánh vòng nhiều hơn? Tương quan hạng khi Openpilot lái cho thấy tốc độ gần như không liên quan (+0,02), độ cong của đường là yếu tố chính (+0,49), và nhìn thấy vạch kẻ thì đỡ hơn (−0,26). Nói cách khác, chạy nhanh không làm xe lắc; vào cua và mất vạch mới làm.

Cần thận trọng với bảng này: mẫu người lái rất nhỏ, 56 và 65 cửa sổ so với hơn một nghìn cửa sổ của Openpilot, và hai bên không đi trên cùng những đoạn đường. Tài xế thường cầm lái đúng ở những chỗ khó, nên con số 0,173 m của người trên đường cong nhiều khả năng bị thổi lên bởi chính sự chọn lọc đó.

### 8.4. Khi nào tài xế giành lại vô-lăng, và có dấu hiệu báo trước không

Tín hiệu thô ghi nhận 81 lần hệ thống bị nhả lái trên 13,5 km. Nhưng một lần can thiệp thật thường sinh ra vài dòng log cách nhau mili-giây: tay đẩy vô-lăng, rồi hệ thống nhả. Sau khi gộp những dòng cách nhau dưới một giây, còn 63 lần can thiệp thật sự, tức 4,7 lần trên mỗi ki-lô-mét lái tự động. Cứ khoảng 210 mét tài xế lại phải chạm vào vô-lăng một lần.

Câu hỏi thú vị hơn: ba giây trước đó có gì khác thường không?

Để trả lời cần một mốc so sánh công bằng, vì tài xế hay can thiệp lúc chạy chậm mà lúc chạy chậm thì mọi chỉ số đều xấu hơn. Nền so sánh vì thế được lấy từ những khung hình cùng dải tốc độ (±3 km/h) mà không có can thiệp nào.

| Dấu hiệu trong 3 giây trước | Trước khi can thiệp | Nền cùng tốc độ | Tỉ lệ |
|---|---|---|---|
| yStd ở mốc 2 giây vượt 0,4 m | 31,2% | 12,1% | 2,59× |
| Mất vạch kẻ (xác suất dưới 0,3) | 70,8% | 54,2% | 1,31× |
| Đang vào cua | 50,0% | 36,8% | 1,36× |

Chỉ một trong ba dấu hiệu thật sự nổi bật. Sự bất định của chính model — yStd — xuất hiện gấp 2,59 lần so với nền trước khi tài xế phải ra tay. Mất vạch và vào cua chỉ tăng nhẹ, và cả hai đều đã phổ biến sẵn trên tuyến đường này nên không phân biệt được gì nhiều.

Ý nghĩa thực tế khá lớn: hệ thống biết trước nó sắp làm điều khiến người lái khó chịu, và thông tin đó đang nằm sẵn trong một trường dữ liệu không được dùng tới. Một cảnh báo sớm dựa trên yStd, hoặc đơn giản là chủ động nhả lái khi yStd vượt ngưỡng, là khả thi ngay với dữ liệu hiện có.

Mẫu là 48 sự kiện có đủ dữ liệu trước đó trên tổng số 63, nên đây là gợi ý mạnh chứ chưa phải kết luận chắc chắn.

### 8.5. Bản đồ biên vận hành

Chia toàn bộ thời gian Openpilot lái thành các ô theo ba trục: tốc độ, hình dạng đường và có nhìn thấy vạch kẻ hay không. Mỗi ô cho biết hệ thống làm việc tốt đến đâu trong điều kiện đó.

| Tốc độ | Đường | Vạch kẻ | Số khung | Sai số 2 s | Đánh vòng |
|---|---|---|---|---|---|
| < 30 km/h | thẳng | có vạch | 6.767 | 0,074 m | 0,012 m |
| < 30 km/h | thẳng | mất vạch | 6.336 | 0,070 m | 0,018 m |
| < 30 km/h | cong | có vạch | 1.929 | 0,127 m | 0,037 m |
| < 30 km/h | cong | mất vạch | 4.368 | 0,189 m | 0,040 m |
| 30–50 km/h | thẳng | có vạch | 10.849 | 0,109 m | 0,013 m |
| 30–50 km/h | thẳng | mất vạch | 1.005 | 0,134 m | 0,030 m |
| 30–50 km/h | cong | có vạch | 1.757 | 0,265 m | 0,030 m |
| 30–50 km/h | cong | mất vạch | 259 | 0,209 m | 0,055 m |
| > 50 km/h | thẳng | có vạch | 319 | 0,083 m | 0,055 m |
| > 50 km/h | thẳng | mất vạch | 55 | 0,107 m | 0,057 m |
| > 50 km/h | cong | có vạch | 126 | 0,388 m | 0,085 m |

Bảng này xác nhận điều mục 8.3 đã gợi ý: trục quyết định không phải tốc độ mà là hình dạng đường. Ở cùng dải dưới 30 km/h, chuyển từ đường thẳng sang đường cong làm sai số tăng từ 0,074 lên 0,127 m khi có vạch, và lên 0,189 m khi mất vạch.

Ô xấu nhất là đường cong trên 50 km/h, sai số 0,388 m và đánh vòng 0,085 m, nhưng chỉ có 126 khung hình và 6 cửa sổ nên chưa đủ để kết luận. Trong dữ liệu gốc chỉ duy nhất ô "trên 50 km/h, đường thẳng, mất vạch" bị đánh dấu mẫu quá ít (3 cửa sổ); những ô còn lại đủ mẫu theo tiêu chí của script, nhưng ba ô cuối bảng vẫn nên đọc một cách dè dặt.

Ba mươi đoạn đáng xem lại bằng video đã được trích ra. Mười lăm đoạn sai số dự đoán lớn nhất tập trung vào đúng năm segment (2, 7, 14, 15 và 45), tất cả ở tốc độ 8–16 km/h — dấu hiệu điển hình của rẽ ở ngã tư, nơi quỹ đạo thật rẽ ngoặt còn model vẫn dự đoán đi thẳng. Sai số lớn nhất là 2,36 mét. Mười lăm đoạn đánh vòng mạnh nhất dẫn đầu bởi segment_46 với biên độ 0,29 mét ở tốc độ 24 km/h.

---

## 9. Kết quả đo ga-phanh trên toàn route

Toàn bộ mục này đo trên cả 52 segment: 52 phút, 16,0 km, trong đó Openpilot giữ ga 14,2 km tương đương 72% thời gian. Số liệu dựa trên 21.940 khung hình có cả radar lẫn thị giác và 27.317 khung hình có xe phía trước.

### 9.1. Tín hiệu gia tốc trên CAN sai tỉ lệ — xác nhận trên 50 segment

| Chỉ số | Giá trị |
|---|---|
| Hệ số aEgo so với gia tốc thật | 3,83 lần (dao động 3,25 – 4,01 giữa các segment) |
| Tương quan | 0,977 (thấp nhất 0,884) |
| Sàn nhiễu của phép ước lượng | 0,058 m/s² so với tín hiệu thật 0,334 m/s² |

Phát hiện này rất ổn định: 50 segment độc lập đều cho cùng một hệ số quanh 3,8.

Nghĩa là bộ lọc Kalman trong `opendbc/car/interfaces.py` báo gia tốc nhỏ hơn thực tế gần bốn lần, và `longcontrol.py` dòng 87 lấy chính nó làm sai số phản hồi. Vòng điều khiển dọc đang nhìn thấy một chiếc xe giảm tốc nhẹ hơn nhiều so với thực tế.

### 9.2. Giữ khoảng cách rất thận trọng

| Chỉ số | Openpilot | Ngưỡng đáng lo |
|---|---|---|
| Khoảng cách thời gian trung vị | 3,40 s | — |
| Phân vị 10 | 1,87 s | — |
| Tỉ lệ dưới 1,0 s | 0,35% | dưới 1 s là bám sát |
| Tỉ lệ dưới 0,6 s | 0,02% | dưới 0,6 s là nguy hiểm |
| TTC trung vị khi đang tiến gần | 17,8 s | — |
| Tỉ lệ TTC dưới 2 s | 0,3% (22 khung hình) | dưới 2 s là sát va chạm |

So với người lái theo cùng dải tốc độ:

| Tốc độ | Openpilot | Người lái |
|---|---|---|
| Dưới 20 km/h | 4,12 s | 4,25 s |
| 20–30 km/h | 3,68 s | 2,85 s |
| 40–60 km/h | 3,22 s | 2,36 s |

Ở tốc độ thấp hai bên như nhau, nhưng khi nhanh hơn thì Openpilot giữ xa hơn người khoảng 30–35%. Đây là hệ quả trực tiếp của chế độ relaxed với `t_follow` = 1,75 giây. Nếu thấy xe bị cắt đầu nhiều thì chuyển sang standard sẽ rút ngắn đáng kể.

### 9.3. Ga-phanh mượt hơn người

| Nhóm | Số cửa sổ | Độ giật (m/s³) | Bám lệnh (m/s²) |
|---|---|---|---|
| Openpilot | 1.626 | 0,162 | 0,043 |
| Người lái | 222 | 0,227 | — |

Tách theo nguồn quyết định của planner cho thấy một trật tự rõ ràng:

| Nguồn | Số cửa sổ | Độ giật | Phanh mạnh nhất |
|---|---|---|---|
| cruise — bám tốc độ cài | 565 | 0,128 | −1,76 m/s² |
| e2e — mạng phanh sớm hơn MPC | 844 | 0,170 | −2,86 m/s² |
| lead0 — bám xe phía trước | 191 | 0,215 | −2,63 m/s² |
| lead1 — bám xe thứ hai | 26 | 0,260 | −2,12 m/s² |

Chạy theo tốc độ cài thì êm nhất, bám xe phía trước thì giật nhất — hợp lý vì lúc đó hệ thống phải phản ứng theo hành vi của xe khác. Mọi lần phanh mạnh nhất đều đến từ hai nguồn e2e và lead, không lần nào từ cruise. Tỉ lệ cửa sổ phải phanh dưới −2 m/s² là 1,3%, và không có lần nào vượt −3 m/s².

### 9.4. Phanh dừng xe: Openpilot dừng sớm và nhẹ, không hề muộn

Sau khi gộp các lần bị đếm trùng do cờ đứng yên nhấp nháy khi xe bò, còn 14 lần dừng hẳn: 9 lần Openpilot giữ ga, 5 lần người lái. Phép đo là quãng đường từ lúc xe còn chạy 18 km/h đến khi dừng hẳn.

| Nhóm | Số lần | Quãng đường | Giảm tốc cần thiết | Giảm tốc đã dùng |
|---|---|---|---|---|
| Openpilot | 9 | 20,0 m | −0,62 m/s² | −1,11 m/s² |
| Người lái | 5 | 7,4 m | −1,69 m/s² | −2,34 m/s² |

Openpilot cần quãng đường gấp 2,7 lần người lái để dừng từ cùng một tốc độ. Nó bắt đầu giảm tốc rất sớm rồi bò vào điểm dừng, trong khi người lái phanh muộn và dứt khoát.

Điều này trả lời câu hỏi bỏ ngỏ từ Phase 3. Bản vá `VF_MILD_DECEL_SCALE` của fork sinh ra để chữa chứng phanh muộn trên VF8 và VF9, và nó làm nhẹ bớt lệnh phanh. Trên VF6 thì triệu chứng ngược lại: xe đã phanh quá sớm và quá nhẹ. Nếu mở rộng bản vá đó cho VF6, vấn đề sẽ nặng thêm chứ không đỡ.

Mẫu còn nhỏ — 9 lần so với 5 lần — nhưng hướng thì rất rõ và nhất quán.

### 9.5. Tài xế hiếm khi can thiệp vào ga-phanh

| Loại can thiệp | Số lần | Trên mỗi km |
|---|---|---|
| Đạp phanh khi Openpilot đang giữ ga | 8 | 0,6 lần/km |
| Đạp ga khi Openpilot đang giữ ga | 0 | 0 |
| Giành lại vô-lăng (đo ở mục 8.4) | 63 | 4,7 lần/km |

Chênh lệch gần tám lần giữa hai con số nói lên nhiều điều: tài xế tin phần ga-phanh hơn hẳn phần lái.

Tám lần đạp phanh đó không giống nhau. Sáu lần có xe phía trước, khoảng cách trung vị 36 mét. Bốn trong số đó rõ ràng thong thả, thời gian tới va chạm 13,8 đến 15,9 giây, tức là tài xế chủ động can thiệp chứ không phải bị dồn. Nhưng hai lần còn lại thì khác: TTC chỉ 4,1 và 3,2 giây, ở tốc độ 20 và 13 km/h. Hai lần đó đáng xem lại bằng video, vì có thể hệ thống thật sự phản ứng chậm.

---

## 10. Những điểm cần lưu ý khi đọc tài liệu này

### 10.1. Phải dùng đúng schema khi đọc log của bản fork

Định dạng capnp lưu enum dưới dạng số, tên chỉ được tra ra lúc đọc. Nếu đọc log của fork bằng schema của bản gốc thì số liệu vẫn đúng nhưng tên có thể sai hoàn toàn. Trường hợp thật đã gặp:

| Chỉ số | Tên theo bản gốc | Tên theo bản fork |
|---|---|---|
| 35 | byd | vinfast |
| 36 | volvo | vinfastVf6 |

Đọc nhầm schema còn khiến một số bản tin biến mất hoặc đổi tên: `liveParameters` bị đọc thành `vehicleParameters`, còn `liveDelay` và `liveTracks` thì không thấy đâu dù thực tế có lần lượt 240 và 1564 bản tin. Ba con số quan trọng trong tài liệu này — độ trễ lái 0,326 giây, tầm nhìn lệnh lái 0,401 giây, và dữ liệu điểm radar ở Phase 4 — chỉ lộ ra sau khi đọc lại bằng schema đúng.

### 10.2. Phạm vi của số liệu

Hai phép đo lớn chạy trên phạm vi hơi khác nhau. Phần lái ở mục 8 dùng 45 trong 52 segment, bảy segment bị loại vì IMU không khớp hoặc xe đứng yên. Phần ga-phanh ở mục 9 dùng cả 52 segment, vì nó không cần dựng lại quỹ đạo nên không cần vòng kiểm định đó. Vì vậy quãng đường nền của hai mục lệch nhau: 13,5 km so với 14,2 km.

Các con số giải thích cơ chế ở Phase 1, 2 và 3 vẫn lấy từ một segment 60 giây, vì mục đích của chúng là minh hoạ chứ không phải thống kê. Cột cuối của bảng ở mục 7 ghi rõ từng dòng thuộc loại nào.

Toàn bộ dữ liệu là nội thành Hà Nội, tốc độ trung vị khoảng 22 km/h. Chưa có dữ liệu chạy liên tục trên 50 km/h.

Về điều kiện ánh sáng: độ sáng trung vị của camera tăng dần từ 0,02 ở đầu route lên 0,44 ở segment cuối, nghĩa là phần cuối hành trình đi vào chỗ thiếu sáng. Nhưng đúng những segment đó lại gần như không lái tự động, nên không thể kết luận gì về hành vi ban đêm.

Vị trí xe trong làn vẫn chưa đo. Đây là một khoảng trống chủ động chứ không phải thiếu dữ liệu: toàn bộ phương pháp ở mục 8 cố ý không dùng vạch kẻ để giữ tính độc lập của đáp án. Dữ liệu thì có sẵn — 15.740 khung hình vừa lái tự động vừa nhìn rõ cả hai vạch, chiếm 46,6% thời gian lái tự động — nên phép đo thiên lệch trái hoàn toàn chạy được, chỉ là chưa chạy.

Cột độ sáng theo từng khung hình trong lần chạy này bị rỗng: bản tin `narrowRoadCameraState` không tồn tại trong schema của fork, nên chỉ còn độ sáng tổng hợp theo segment. Không con số nào trong tài liệu phụ thuộc vào cột đó.

Phần phanh dừng xe chỉ có 14 mẫu (9 của Openpilot, 5 của người lái). Hướng rất rõ nhưng cần thêm dữ liệu để chắc.

### 10.3. Những câu hỏi còn bỏ ngỏ

- Firmware panda đang chạy trên xe thực thi những giới hạn nào? Kho mã công khai không trả lời được.
- Vì sao hệ thống lái chỉ thực hiện 44% góc được lệnh, và tỉ lệ đó có giữ nguyên ở tốc độ cao không?
- Thiên lệch trái của bản vá VinFast có thật sự đẩy xe lệch trong làn không? Dữ liệu đã đủ để đo (mục 10.2) nhưng phép đo chưa được chạy.
- VF6 có mắc chứng phanh muộn như VF8 và VF9 không? Câu này đã có câu trả lời sơ bộ ở mục 9.4: ngược lại, VF6 phanh sớm và nhẹ, nên mở rộng bản vá `VF_MILD_DECEL_SCALE` cho VF6 sẽ phản tác dụng. Cần thêm mẫu để khẳng định.

### 10.4. Những chỗ đã sai và đã được sửa

Tài liệu này là bản đã sửa của một chuỗi phân tích dài. Liệt kê lại những chỗ từng sai để người đọc biết mức độ tin cậy của phương pháp, và để không ai vô tình trích lại con số cũ.

| Đã từng kết luận | Sự thật sau khi sửa | Cái gì phát hiện ra |
|---|---|---|
| Model tự tin thái quá, yStd nhỏ hơn sai số thật 2,2 lần | Ngược lại: yStd hơi rộng hơn sai số thật (tỉ lệ 0,77–0,92) | Bài kiểm tra phá hoại: cố tình đảo dấu quỹ đạo thật thì sai số lại GIẢM, chứng tỏ dấu đang sai |
| Đọc log bằng schema của openpilot gốc | Phải dùng schema của fork; nếu không, safetyModel hiện ra là volvo và ba bản tin quan trọng biến mất | Đối chiếu tên hãng xe ở chỉ số 35–36 trong enum |
| Độ trễ lái 0,125 s | 0,401 s — vì liveDelay chỉ đọc được bằng schema đúng | Cùng nguyên nhân trên |
| Plan có 5 giả thuyết quỹ đạo | Chỉ có 1; tham số in_N = 0 tắt cơ chế đa giả thuyết cho plan (lead thì vẫn có 2) | Đọc lại lời gọi parse_mdn trong mã nguồn |
| Bù 1,52 m giữa radar và camera là hợp lý | Không có độ lệch cố định nào: sai lệch thay đổi theo khoảng cách, từ −1,50 m ở gần đến +0,11 m ở xa | Chạy trên cả route thay vì một segment |
| Thiên lệch trái chiếm một phần ba lệnh lái | Trung vị chỉ 9%, nhưng trên đoạn gần thẳng thì lên tới 430% lệnh của model | Tính lại có kèm hệ số suy giảm theo độ cong |
| Tín hiệu gia tốc aEgo hỏng, tương quan 0,26 | Tương quan 0,977, chỉ sai tỉ lệ 3,83 lần | So với đạo hàm đã lọc thay vì đạo hàm thô |
| 129 lần tài xế can thiệp | 63 lần — mỗi lần sinh vài dòng log cách nhau mili-giây | Gộp các dòng cách nhau dưới 1 giây |

Điểm chung của cả tám chỗ: không chỗ nào được phát hiện bằng cách đọc lại kết quả, mà bằng cách hỏi "nếu kết luận này sai thì phép đo sẽ phản ứng thế nào" rồi đi làm đúng phép thử đó.

---

## 11. Thuật ngữ

| Thuật ngữ | Nghĩa |
|---|---|
| `desiredCurvature` | độ cong quỹ đạo mong muốn, đơn vị 1/m, là nghịch đảo bán kính cua |
| `curvature` | độ cong xe đang thực sự có, suy từ góc vô-lăng qua mô hình xe |
| `latActive` / `longActive` | hệ thống có đang điều khiển lái / ga-phanh hay không |
| `steerRatio` | tỉ số truyền lái: quay vô-lăng bao nhiêu độ thì bánh xe quay một độ |
| `curvature_factor` | hệ số quy đổi góc bánh xe sang độ cong, đổi theo tốc độ do lốp trượt ngang |
| `angleOffsetDeg` | vô-lăng lệch tâm bao nhiêu độ khi xe đi thẳng |
| `roll` | độ nghiêng ngang của mặt đường; đường nghiêng thì xe tự trôi nên phải bù |
| `yStd`, `xStd` | độ lệch chuẩn mà model tự khai cho từng đại lượng nó dự đoán |
| feedforward | phần lệnh tính thẳng từ vật lý, không chờ sai số; đối lập với feedback |
| PI, PID | bộ điều khiển phản hồi: P theo sai số hiện tại, I cộng dồn quá khứ, D theo tốc độ đổi |
| deadband, vùng chết | ngưỡng dưới đó thì bỏ qua sai số, tránh rung lắc quanh điểm cân bằng |
| MPC | điều khiển dự báo theo mô hình: mô phỏng tương lai, tối ưu, chỉ dùng bước đầu |
| ACADOS | thư viện C giải bài toán tối ưu có ràng buộc trong thời gian thực |
| SQP_RTI | chỉ chạy một vòng lặp Newton mỗi chu kỳ thay vì lặp đến hội tụ |
| HPIPM | bộ giải quy hoạch toàn phương, cho lời giải dùng được cả khi bị dừng sớm |
| ràng buộc mềm | ràng buộc được phép vi phạm nhưng bị phạt rất nặng |
| `x_obstacle` | vị trí vật cản quy đổi: xe trước, hoặc vật ảo đại diện cho tốc độ cài đặt |
| `t_follow` | thời gian giãn cách mong muốn với xe trước, đổi theo chế độ lái |
| độ giật (jerk) | đạo hàm của gia tốc; thứ quyết định cảm giác êm hay xóc |
| `dRel`, `yRel`, `vRel` | khoảng cách, lệch ngang và vận tốc tương đối của xe phía trước |
| `aLeadK`, `vLeadK` | gia tốc và vận tốc xe trước sau khi lọc Kalman |
| Kalman filter | thuật toán ước lượng đại lượng ẩn từ chuỗi quan sát nhiễu |
| Laplacian pdf | hàm mật độ dạng exp(−\|x−μ\|/b); đuôi dày hơn Gauss nên ít bị ngoại lai đánh lừa |
| VRU | vulnerable road user — người đi bộ, xe đạp, xe máy |
| TTC | time to collision — thời gian còn lại đến va chạm nếu giữ nguyên tốc độ |
| panda | bo mạch phần cứng giữa thiết bị và xe, lọc bản tin CAN |
| `controlsAllowed` | cờ trong panda cho biết có cho phép gửi lệnh điều khiển xuống xe không |
| `softDisabling` | trạng thái đếm ngược 3 giây trước khi tắt, cho phép hồi phục |
| capnp | định dạng nhị phân dùng cho mọi bản tin nội bộ và cho log |
