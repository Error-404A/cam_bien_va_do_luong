# Đo lường, Hiệu chỉnh và Lọc tín hiệu Cảm biến Quán tính MPU6050

> **Môn:** Cảm biến và đo lường cho Robot  
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

*Báo cáo chi tiết: xem file `cảm_biến_và_đo_lường.pdf` trong repository.*