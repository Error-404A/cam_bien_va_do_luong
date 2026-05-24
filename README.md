# Đo lường, Hiệu chỉnh và Lọc tín hiệu Cảm biến Quán tính MPU6050

> **Môn:** Kỹ thuật Cảm biến  
> **Sinh viên:** Lê Quang Khải — MSSV: 20422878  
> **Lớp:** RBE3042-2 — Nhóm: 14  
> **Giảng viên:** TS. Vũ Quốc Tuấn  
> **Trường:** Đại học Công Nghệ — ĐHQGHN

---

## Tổng quan

Repository này chứa toàn bộ mã nguồn, dữ liệu thực nghiệm và kết quả phân tích của **6 bài thực hành** sử dụng cảm biến IMU MPU6050 kết hợp Arduino và MATLAB.

Hệ thống thu thập dữ liệu từ MPU6050 qua giao tiếp I2C (400 kHz), xuất CSV qua Serial Monitor, sau đó xử lý và trực quan hóa bằng MATLAB.

```
MPU6050 ──I2C──► Arduino ──CSV──► MATLAB (phan_1.m ~ phan_6.m)
  (cảm biến)    (thu thập)         (phân tích + đồ thị)
```

---

##  Cấu trúc thư mục

```
cam_bien_va_do_luong/
│
├── arduino/
│   └── MPU6050_fixed.ino          # Firmware Arduino: menu 7 bài, I2C, CF+KF
│
├── phan_1.m                   				# Bài 1: Đặc tính tĩnh
├── phan_2.m                   				# Bài 2: Hiệu chỉnh offset
├── phan_3.m                   				# Bài 3: Ảnh hưởng trọng trường
├── phan_4.m                   				# Bài 4: Complementary & Kalman Filter
├── phan_5.m                   				# Bài 5: Shock detection & FFT
└── phan_6.m                   				# Bài 6: Ước lượng góc Roll/Pitch/Yaw
│
├── bai1_data.txt              				# 500 mẫu tĩnh (Ax,Ay,Az,Gx,Gy,Gz)
├── bai2_gyro.txt              				# 500 mẫu gyro đứng yên
├── bai2_accel_pos1~6.txt      				# 200 mẫu/vị trí, 6 vị trí hiệu chỉnh
├── bai3_data.txt              				# Dữ liệu 6 góc nghiêng (0°~90°)
├── bai4_data.txt              				# 1800 mẫu CF+KF (100 Hz, ~18s)
├── bai5_shock_data.txt        				# Shock detection (500 Hz)
├── bai5_fft_data.txt          				# 512 mẫu dao động (500 Hz)
├── bai_6.txt                  				# 3500 mẫu Roll/Pitch/Yaw (100 Hz, 35s)
└── Kết_quả_thí_nghiệm.txt     				# Tổng hợp kết quả số của tất cả 6 bài
│
├── phan1.png							# Đồ thị đặc tính tĩnh
├── phan2_gyro.png  	        				# Gyro trước/sau hiệu chỉnh
├── phan2_accel.png	        				# Accel 6-position
├── phan3.png	               					# Phân bổ gia tốc trọng trường
├── phan4_đồ_thị_Roll_&_Pitch_Raw_CF_KF_theo_thời_gian.png	# So sánh 3 phương pháp
├── phan4_phổ FFT						# Phổ FFT góc Roll
├── phan4_histogram						# Phân bố nhiễu sau các bộ lọc
├── phan4_sai lệch CF − KF theo thời gian			# Sai lệch CF − KF theo thời gian
├── phan5_toàn bộ tín hiệu shock				# Tổng quan tín hiệu phát hiện shock
├── phan5_zoom sự kiện shock					# Chi tiết từng sự kiện shock
├── phan5_tín hiệu dao động trong mien thời gian		# Tín hiệu dao động trong miền thời gian
├── phan5_phổ FFT						# Phân thích phổ FFT tín hiệu dao động
├── bai6_overview.png          				# Roll/Pitch/Yaw tổng quan
└── bai6_detail.png            				# Chi tiết Roll+Pitch+Yaw drift
│
└── README.md
```

---

##  Phần cứng & Phần mềm

| Thành phần | Chi tiết |
|---|---|
| **Cảm biến** | MPU6050 — 6-DOF IMU (Accel 3 trục + Gyro 3 trục) |
| **Giao tiếp** | I2C, địa chỉ `0x68` (AD0=GND), tốc độ 400 kHz |
| **Vi điều khiển** | Arduino Uno/Mega/Nano |
| **IDE thu thập** | Arduino IDE, baud rate 115200 |
| **Phần mềm phân tích** | MATLAB R2021a trở lên |
| **Dải đo Accel** | ±2g — Scale Factor: 16384 LSB/g |
| **Dải đo Gyro** | ±250°/s — Scale Factor: 131 LSB/(°/s) |
| **Tần số lấy mẫu** | 100 Hz (Bài 1–4, 6) / 500 Hz (Bài 5) |

---

##  Hướng dẫn sử dụng

### 1. Nạp firmware Arduino

```
Mở file: arduino/MPU6050_fixed.ino
Nạp vào Arduino, mở Serial Monitor (115200 baud)
Gửi số 1–6 để chọn bài thực hành, gửi 'q' để dừng
```

### 2. Thu thập dữ liệu

Firmware xuất dữ liệu dạng CSV theo thời gian thực. Copy nội dung từ Serial Monitor và lưu vào file `.txt` tương ứng trong thư mục `data/`.

### 3. Chạy phân tích MATLAB

```matlab
% Đặt working directory vào thư mục matlab/
% Chạy từng script theo thứ tự:
run('phan_1.m')   % Bài 1
run('phan_2.m')   % Bài 2
% ... tương tự đến phan_6.m
```

Các đồ thị được lưu tự động vào thư mục hiện tại dưới dạng `.png`.

---

##  Nội dung 6 bài thực hành

### Bài 1 — Đo lường cơ bản & Đặc tính tĩnh

Thu 500 mẫu khi cảm biến đứng yên, đánh giá zero-offset, nhiễu (STD) và gia tốc tổng hợp |a|.

| Kênh | Mean | STD | Đánh giá |
|------|------|-----|----------|
| Ax (g) | +0.01875 | 0.002891 |  OK |
| Ay (g) | −0.01150 | 0.002646 |  OK |
| Az (g) | +0.99233 | 0.003067 |  OK |
| Gx (°/s) | +0.12481 | 0.017144 |  OK |
| Gy (°/s) | −0.08755 | 0.017464 |  OK |
| Gz (°/s) | +0.05265 | 0.016076 |  OK |
| **\|a\|** | **0.99259 g** | — |  Sai lệch 0.74% < 3% |

---

### Bài 2 — Hiệu chỉnh Offset (Calibration)

**2A — Gyroscope:** Đứng yên 5s, tính offset trung bình → áp dụng vào firmware.

| Trục | Offset (°/s) | STD |
|------|-------------|-----|
| Gx | +0.309 | 0.056 |
| Gy | −0.199 | 0.059 |
| Gz | +0.847 | 0.058 |

**2B — Accelerometer (6-position):** Đo lần lượt 6 hướng, tính offset theo công thức:

```
offset_ax = -(mean_ax_up + mean_ax_down) / 2
```

Offset tính được: `ax = +0.018g`, `ay = −0.025g`, `az = +0.262g`

---

### Bài 3 — Ảnh hưởng gia tốc trọng trường

Nghiêng cảm biến từ 0° đến 90°, so sánh giá trị đo với lý thuyết `Ax = sin(θ)`, `Az = cos(θ)`.

| Góc | \|a\| đo (g) | Sai lệch |
|-----|------------|---------|
| 0° | 1.0016 | 0.16%   |
| 30° | 1.0006 | 0.06%  |
| 45° | 1.0018 | 0.18%  |
| 90° | 1.0017 | 0.17%  |

> Tất cả 6 góc đều đạt sai lệch < 0.2% — **xuất sắc** so với ngưỡng yêu cầu 3%.

---

### Bài 4 — Bộ lọc tín hiệu (CF & KF)

So sánh 3 phương pháp ước lượng góc trong khi xoay cảm biến (~18s, 100Hz):

| Phương pháp | Roll STD (°) | Chống drift | Độ phức tạp |
|-------------|-------------|-------------|-------------|
| Raw Accel | 12.16 | Thấp |
| Complementary (α=0.98) | 12.11 |  Tốt | Thấp |
| Kalman Filter | 13.73 |  Rất tốt | Trung bình |

**Tham số KF:** `Q_angle=0.001`, `Q_bias=0.003`, `R_measure=0.03`

> Sai lệch CF vs KF: Max Roll = 13.07°, RMS = 5.48° — phản ánh chiến lược xử lý chuyển động khác nhau.

---

### Bài 5 — Đo Shock & Phân tích dao động (FFT)

**Shock detection** (500 Hz, ngưỡng |a|−1g > 2g):

| Sự kiện | Biên độ đỉnh | Xung | Settling time | f tự nhiên |
|---------|-------------|------|---------------|-----------|
| Shock #1 | 4.370 g | 6 ms | 74 ms | ~10.4 Hz |
| Shock #2 | 6.145 g | 10 ms | 112 ms | ~8.8 Hz |

**FFT** (512 mẫu, 500 Hz):

| Hạng | Tần số (Hz) | Biên độ (g) |
|------|------------|------------|
| 1 | 25.4 | 0.1121 |
| 2 | 49.8 | 0.0508 |
| 3 | 9.8 | 0.0231 |

> LPF Butterworth bậc 4 (fc=80Hz) loại bỏ hiệu quả các thành phần > 80Hz.

---

### Bài 6 — Ước lượng góc Roll / Pitch / Yaw

Chuỗi thí nghiệm 35s: yên tĩnh → Roll 45° → giữ → về 0° → Pitch 30° → Yaw tự do.

| Chỉ tiêu | Kết quả | Đánh giá |
|---------|---------|---------|
| STD khi giữ Roll=45° | 0.059° |  Rất tốt (< 0.5°) |
| Sai số tại Roll=45° | 0.001° |  Xuất sắc |
| Thời gian hội tụ về 0° | 1.96 s |  Xuất sắc (< 2s) |
| Latency KF vs Accel | 40 ms |  Chấp nhận được (< 50ms) |
| Sai số Pitch=30° | 0.017° |  Rất tốt |
| **Yaw drift** | **10.05°/s** |  Cần Magnetometer |

> Góc Yaw bị drift 60.33° sau 6s do không có magnetometer. Đây là hạn chế cơ bản của hệ thống 6-DOF.

---

## Một số đồ thị kết quả

<table>
<tr>
<td><b>Bài 1 — Đặc tính tĩnh</b><br><img src="results/bai1_static.png" width="380"/></td>
<td><b>Bài 3 — Gia tốc trọng trường</b><br><img src="results/bai3_gravity.png" width="380"/></td>
</tr>
<tr>
<td><b>Bài 6 — Roll/Pitch/Yaw tổng quan</b><br><img src="results/bai6_overview.png" width="380"/></td>
<td><b>Bài 6 — Chi tiết Roll + Yaw drift</b><br><img src="results/bai6_detail.png" width="380"/></td>
</tr>
</table>

---

## Các thông số quan trọng trong firmware

```cpp
// Địa chỉ I2C
#define MPU_ADDR        0x68

// Scale factors
#define ACCEL_SCALE_2G  16384.0f   // ±2g
#define GYRO_SCALE_250  131.0f     // ±250°/s

// Kalman Filter
float Q_angle   = 0.001f;
float Q_bias    = 0.003f;
float R_measure = 0.03f;

// Complementary Filter
float cf_alpha  = 0.98f;

// Shock threshold
#define SHOCK_THRESHOLD  2.0f      // g
```

---

## Ghi chú kỹ thuật

- **MPU6050 vs MPU6500:** Register map tương tự nhau. Điểm khác biệt chính: `WHO_AM_I = 0x68` (MPU6050) vs `0x70` (MPU6500), và công thức nhiệt độ: `T = raw/340.0 + 36.53` (MPU6050).
- **Drift Yaw:** Không thể khắc phục với MPU6050 đơn thuần. Giải pháp: dùng MPU9250 (tích hợp từ kế AK8963) hoặc ghép thêm module HMC5883L.
- **Hiệu chỉnh trục Z:** Pos1 và Pos2 (az up/down) có sai lệch lớn hơn các trục X, Y do khó đặt cảm biến chính xác 90° khi lật ngược.
- **STD của KF lớn hơn raw:** Bình thường khi đo trong chuyển động tự do — KF theo sát động học thực, không phải lỗi bộ lọc.

---
*Báo cáo chi tiết: xem file `cảm_biến_và_đo_lường.pdf` trong repository.*