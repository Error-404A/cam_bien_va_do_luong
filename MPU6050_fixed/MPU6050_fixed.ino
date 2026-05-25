#include <Wire.h>
#include <math.h>

// ================================================================
//  REGISTER MAP MPU6050
// ================================================================
#define MPU_ADDR           0x68    // AD0=GND -> 0x68 | AD0=VCC -> 0x69

#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A    // DLPF Config cho CA Accel va Gyro
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_INT_ENABLE     0x38
#define REG_INT_STATUS     0x3A
#define REG_ACCEL_XOUT_H   0x3B
#define REG_TEMP_OUT_H     0x41
#define REG_GYRO_XOUT_H    0x43
#define REG_PWR_MGMT_1     0x6B
#define REG_PWR_MGMT_2     0x6C
#define REG_WHO_AM_I       0x75    // Dia chi thanh ghi WHO_AM_I (MPU6050 tra ve 0x68)

// ================================================================
//  SCALE FACTORS (Sensitivity Scale Factor)
// ================================================================
#define ACCEL_SCALE_2G    16384.0f   // +-2g  -> 16384 LSB/g
#define ACCEL_SCALE_4G     8192.0f   // +-4g
#define ACCEL_SCALE_8G     4096.0f   // +-8g
#define GYRO_SCALE_250      131.0f   // +-250 deg/s -> 131 LSB/(deg/s)
#define GYRO_SCALE_500       65.5f   // +-500 deg/s
#define GYRO_SCALE_1000      32.8f   // +-1000 deg/s

#define RAD_TO_DEG         57.2957795f
#define DEG_TO_RAD          0.0174533f

// ================================================================
//  CAU TRUC DU LIEU
// ================================================================

// Du lieu cam bien tuc thoi
struct SensorData {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  int16_t temp_raw;
  float   ax_g,  ay_g,  az_g;       // quy ve g
  float   gx_dps, gy_dps, gz_dps;   // do/giay
  float   temp_c;                   // nhiet do deg C
};

// Thong so hieu chinh
struct CalibData {
  float ax_off, ay_off, az_off;      // offset accel (g)
  float gx_off, gy_off, gz_off;      // offset gyro (deg/s)
  bool  done;
};

// Kalman 1D (cho mot truc goc)
struct KalmanFilter {
  float angle;        // goc uoc luong
  float bias;         // uoc luong bias gyro
  float P[2][2];      // ma tran hiep phuong sai
  float Q_angle;      // nhieu qua trinh - goc
  float Q_bias;       // nhieu qua trinh - bias
  float R_measure;    // nhieu do luong (accel)
};

// ================================================================
//  BIEN TOAN CUC
// ================================================================
SensorData  imu;
CalibData   calib      = {0,0,0, 0,0,0, false};
KalmanFilter kf_roll   = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
KalmanFilter kf_pitch  = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};

float cf_alpha         = 0.98f;
float cf_roll          = 0.0f;
float cf_pitch         = 0.0f;
float kf_roll_out      = 0.0f;
float kf_pitch_out     = 0.0f;

unsigned long last_us  = 0;
float dt               = 0.01f;

// Shock detection
#define SHOCK_THRESHOLD  2.0f      // g (nguong phat hien shock, lenh chenh so voi 1g)

// ================================================================
//  HAM I2C CO BAN
// ================================================================

// [FIX #1] Da them Wire.write(data) - truoc do bien 'data' khong bao gio duoc gui
void mpu_write(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);   // <-- FIX: dong nay bi thieu trong phien ban cu, khien
                      //          toan bo cau hinh (DLPF, sample rate, dai do)
                      //          khong duoc nap vao chip
  Wire.endTransmission();
}

uint8_t mpu_read_byte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0;
}

// Doc len byte lien tiep tu reg vao buf[]
void mpu_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, len, (uint8_t)true);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.available() ? Wire.read() : 0;
  }
}

// ================================================================
//  KHOI TAO MPU6050
// ================================================================
void mpu_setup() {
  // Kiem tra WHO_AM_I
  uint8_t who = mpu_read_byte(REG_WHO_AM_I);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);

  // [FIX #2] Sua lai nhan dien WHO_AM_I cho dung voi MPU6050:
  //   - MPU6050 tra ve 0x68 (gia tri dung, da xac nhan theo datasheet)
  //   - 0x70 la WHO_AM_I cua MPU6500, KHONG phai MPU6050
  //   - 0x71 la WHO_AM_I cua MPU9250
  if      (who == 0x68) Serial.println(F("Cam bien: MPU6050 - OK"));
  else if (who == 0x70) Serial.println(F("[WARN] WHO_AM_I=0x70: Day la MPU6500, khong phai MPU6050!"));
  else if (who == 0x71) Serial.println(F("[WARN] WHO_AM_I=0x71: Day la MPU9250, khong phai MPU6050!"));
  else                  Serial.println(F("[WARN] WHO_AM_I khong nhan ra, kiem tra ket noi I2C..."));

  // Reset chip
  mpu_write(REG_PWR_MGMT_1, 0x80);
  delay(150);

  // Wake up + dung PLL clock
  mpu_write(REG_PWR_MGMT_1, 0x01);
  delay(50);

  // DLPF Gyro & Accel: bandwidth ~44 Hz (CONFIG register = 3)
  mpu_write(REG_CONFIG, 0x03);

  // Gyro: Full-Scale +-250 deg/s (GYRO_CONFIG = 0x00)
  mpu_write(REG_GYRO_CONFIG, 0x00);

  // Accel: Full-Scale +-2g (ACCEL_CONFIG = 0x00)
  mpu_write(REG_ACCEL_CONFIG, 0x00);

  // Sample Rate = 1000 / (1 + SMPLRT_DIV) = 100 Hz
  mpu_write(REG_SMPLRT_DIV, 9);

  Serial.println(F("MPU6050 OK: +-2g / +-250dps / 100Hz / DLPF 44Hz"));
}

// ================================================================
//  DOC TAT CA 14 BYTE (Accel + Temp + Gyro)
//  Tu dong ap dung offset neu calib.done == true
// ================================================================
void read_sensor() {
  uint8_t buf[14];
  mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);

  imu.ax_raw   = (int16_t)((buf[0]  << 8) | buf[1]);
  imu.ay_raw   = (int16_t)((buf[2]  << 8) | buf[3]);
  imu.az_raw   = (int16_t)((buf[4]  << 8) | buf[5]);
  imu.temp_raw = (int16_t)((buf[6]  << 8) | buf[7]);
  imu.gx_raw   = (int16_t)((buf[8]  << 8) | buf[9]);
  imu.gy_raw   = (int16_t)((buf[10] << 8) | buf[11]);
  imu.gz_raw   = (int16_t)((buf[12] << 8) | buf[13]);

  // Quy doi sang don vi vat ly
  imu.ax_g    = imu.ax_raw / ACCEL_SCALE_2G;
  imu.ay_g    = imu.ay_raw / ACCEL_SCALE_2G;
  imu.az_g    = imu.az_raw / ACCEL_SCALE_2G;
  imu.gx_dps  = imu.gx_raw / GYRO_SCALE_250;
  imu.gy_dps  = imu.gy_raw / GYRO_SCALE_250;
  imu.gz_dps  = imu.gz_raw / GYRO_SCALE_250;

  imu.temp_c  = (imu.temp_raw / 340.0f) + 36.53f;

  // Ap dung offset hieu chinh
  if (calib.done) {
    imu.ax_g   += calib.ax_off;
    imu.ay_g   += calib.ay_off;
    imu.az_g   += calib.az_off;
    imu.gx_dps -= calib.gx_off;
    imu.gy_dps -= calib.gy_off;
    imu.gz_dps -= calib.gz_off;
  }
}

// ================================================================
//  KALMAN FILTER 1D - CAP NHAT MOT TRUC
// ================================================================
float kalman_update(KalmanFilter* kf, float accel_angle, float gyro_rate, float dt_s) {
  // --- Buoc Predict (Du bao) ---
  float rate  = gyro_rate - kf->bias;
  kf->angle  += dt_s * rate;

  kf->P[0][0] += dt_s * (dt_s * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
  kf->P[0][1] -= dt_s * kf->P[1][1];
  kf->P[1][0] -= dt_s * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt_s;

  // --- Buoc Update (Cap nhat tu Accel) ---
  float S  = kf->P[0][0] + kf->R_measure;
  float K0 = kf->P[0][0] / S;
  float K1 = kf->P[1][0] / S;

  float y  = accel_angle - kf->angle;
  kf->angle += K0 * y;
  kf->bias  += K1 * y;

  float P00_tmp = kf->P[0][0];
  float P01_tmp = kf->P[0][1];
  kf->P[0][0] -= K0 * P00_tmp;
  kf->P[0][1] -= K0 * P01_tmp;
  kf->P[1][0] -= K1 * P00_tmp;
  kf->P[1][1] -= K1 * P01_tmp;

  return kf->angle;
}

// ================================================================
//  MENU CHINH
// ================================================================
void print_menu() {
  Serial.println(F("\n========================================"));
  Serial.println(F("     MPU6050 - KY THUAT CAM BIEN       "));
  Serial.println(F("========================================"));
  Serial.println(F(" 1 - Bai 1 : Do luong co ban & Dac tinh tinh"));
  Serial.println(F(" 2 - Bai 2 : Hieu chinh Offset (Calibration)"));
  Serial.println(F(" 3 - Bai 3 : Anh huong gia toc trong truong"));
  Serial.println(F(" 4 - Bai 4 : Bo loc (Complementary & Kalman)"));
  Serial.println(F(" 5 - Bai 5 : Do shock & Lay mau dao dong (FFT)"));
  Serial.println(F(" 6 - Bai 6 : Uoc luong goc Roll/Pitch/Yaw"));
  Serial.println(F(" 7 - Bai 7 : Bai tap tong hop"));
  Serial.println(F(" 0 - Hien thi lai menu"));
  Serial.println(F("========================================"));
  Serial.print(F("Calib: "));
  Serial.println(calib.done ? F("[CO] - Da hieu chinh") : F("[CHUA] - Chua hieu chinh"));
  Serial.println(F("Gui so 1-7 de chon bai:"));
}

// ================================================================
//  TIEN ICH: doi ky tu tu Serial
// ================================================================
char wait_serial() {
  while (!Serial.available()) delay(20);
  char c = (char)Serial.read();
  while (Serial.available()) Serial.read();
  return c;
}

void flush_serial() {
  delay(10);
  while (Serial.available()) Serial.read();
}

// ================================================================
//  BAI 1: DO LUONG CO BAN & DAC TINH TINH
// ================================================================
void task1_static() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  BAI 1: DO LUONG CO BAN & DAC TINH TINH"));
  Serial.println(F("========================================"));
  Serial.println(F("Dat cam bien nam yen tren mat phang bang."));
  Serial.println(F("Dang chuan bi lay 500 mau (khong di chuyen)..."));
  delay(3000);

  const int N = 500;
  double sum_ax=0,sum_ay=0,sum_az=0;
  double sum_gx=0,sum_gy=0,sum_gz=0;
  double sq_ax=0,sq_ay=0,sq_az=0;
  double sq_gx=0,sq_gy=0,sq_gz=0;

  Serial.println(F("ax,ay,az,gx,gy,gz"));

  for (int i = 0; i < N; i++) {
    uint8_t buf[14];
    mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);

    float ax = (int16_t)((buf[0]<<8)|buf[1]) / ACCEL_SCALE_2G;
    float ay = (int16_t)((buf[2]<<8)|buf[3]) / ACCEL_SCALE_2G;
    float az = (int16_t)((buf[4]<<8)|buf[5]) / ACCEL_SCALE_2G;
    float gx = (int16_t)((buf[8]<<8)|buf[9]) / GYRO_SCALE_250;
    float gy = (int16_t)((buf[10]<<8)|buf[11]) / GYRO_SCALE_250;
    float gz = (int16_t)((buf[12]<<8)|buf[13]) / GYRO_SCALE_250;

    sum_ax+=ax; sum_ay+=ay; sum_az+=az;
    sum_gx+=gx; sum_gy+=gy; sum_gz+=gz;
    sq_ax+=ax*ax; sq_ay+=ay*ay; sq_az+=az*az;
    sq_gx+=gx*gx; sq_gy+=gy*gy; sq_gz+=gz*gz;

    Serial.print(ax,4); Serial.print(',');
    Serial.print(ay,4); Serial.print(',');
    Serial.print(az,4); Serial.print(',');
    Serial.print(gx,4); Serial.print(',');
    Serial.print(gy,4); Serial.print(',');
    Serial.println(gz,4);

    delay(10);
  }

  float m_ax = sum_ax/N, m_ay = sum_ay/N, m_az = sum_az/N;
  float m_gx = sum_gx/N, m_gy = sum_gy/N, m_gz = sum_gz/N;
  float s_ax = sqrt(sq_ax/N - (double)m_ax*m_ax);
  float s_ay = sqrt(sq_ay/N - (double)m_ay*m_ay);
  float s_az = sqrt(sq_az/N - (double)m_az*m_az);
  float s_gx = sqrt(sq_gx/N - (double)m_gx*m_gx);
  float s_gy = sqrt(sq_gy/N - (double)m_gy*m_gy);
  float s_gz = sqrt(sq_gz/N - (double)m_gz*m_gz);
  float mag  = sqrt(m_ax*m_ax + m_ay*m_ay + m_az*m_az);

  Serial.println(F("\n--- KET QUA ---"));
  Serial.println(F("[ACCELEROMETER] (don vi: g, ly tuong: Ax=0, Ay=0, Az=1)"));
  Serial.print(F("  Mean : Ax=")); Serial.print(m_ax,5);
  Serial.print(F("  Ay="));        Serial.print(m_ay,5);
  Serial.print(F("  Az="));        Serial.println(m_az,5);
  Serial.print(F("  STD  : Ax=")); Serial.print(s_ax,6);
  Serial.print(F("  Ay="));        Serial.print(s_ay,6);
  Serial.print(F("  Az="));        Serial.println(s_az,6);

  Serial.println(F("[GYROSCOPE] (don vi: deg/s, ly tuong: 0)"));
  Serial.print(F("  Mean : Gx=")); Serial.print(m_gx,5);
  Serial.print(F("  Gy="));        Serial.print(m_gy,5);
  Serial.print(F("  Gz="));        Serial.println(m_gz,5);
  Serial.print(F("  STD  : Gx=")); Serial.print(s_gx,6);
  Serial.print(F("  Gy="));        Serial.print(s_gy,6);
  Serial.print(F("  Gz="));        Serial.println(s_gz,6);

  Serial.println(F("[DANH GIA DAC TINH TINH]"));
  Serial.print(F("  |a| tong = ")); Serial.print(mag,5);
  Serial.print(F(" g  (ly tuong: 1.000 g)  Sai lech: "));
  Serial.print(abs(mag-1.0f)*100,2); Serial.println(F("%"));

  if (abs(m_ax) < 0.05f && abs(m_ay) < 0.05f)
    Serial.println(F("  [OK] Zero-offset Accel X,Y trong gioi han."));
  else
    Serial.println(F("  [!] Zero-offset Accel lon, can hieu chinh (Bai 2)."));

  if (abs(m_gx)<0.5f && abs(m_gy)<0.5f && abs(m_gz)<0.5f)
    Serial.println(F("  [OK] Zero-offset Gyro chap nhan duoc."));
  else
    Serial.println(F("  [!] Gyro offset lon, can hieu chinh (Bai 2)."));

  if (abs(mag-1.0f) < 0.03f)
    Serial.println(F("  [OK] |a| trong gioi han 3%."));
  else
    Serial.println(F("  [!] |a| lech >3%! Can hieu chinh gain Accel."));

  Serial.println(F("\nBai 1 hoan tat. Gui '0' de ve menu."));
}

// ================================================================
//  BAI 2A: HIEU CHINH GYROSCOPE (Static - 5 giay)
// ================================================================
void task2_calib_gyro() {
  Serial.println(F("\n[2A] HIEU CHINH GYROSCOPE"));
  Serial.println(F("Dat cam bien YEN LANG hoan toan trong 5 giay..."));
  Serial.println(F("Nhan phim bat ky de bat dau:"));
  wait_serial();

  const int N = 500;
  double sgx=0,sgy=0,sgz=0;
  double sq_gx=0,sq_gy=0,sq_gz=0;

  Serial.println(F("gx,gy,gz"));

  for (int i = 0; i < N; i++) {
    uint8_t buf[14];
    mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);
    float gx = (int16_t)((buf[8]<<8)|buf[9])   / GYRO_SCALE_250;
    float gy = (int16_t)((buf[10]<<8)|buf[11])  / GYRO_SCALE_250;
    float gz = (int16_t)((buf[12]<<8)|buf[13])  / GYRO_SCALE_250;
    sgx+=gx; sgy+=gy; sgz+=gz;
    sq_gx+=gx*gx; sq_gy+=gy*gy; sq_gz+=gz*gz;

    Serial.print(gx, 4); Serial.print(',');
    Serial.print(gy, 4); Serial.print(',');
    Serial.println(gz, 4);
    delay(10);
  }

  calib.gx_off = sgx/N;
  calib.gy_off = sgy/N;
  calib.gz_off = sgz/N;

  float std_gx = sqrt(sq_gx/N - (double)calib.gx_off*calib.gx_off);
  float std_gy = sqrt(sq_gy/N - (double)calib.gy_off*calib.gy_off);
  float std_gz = sqrt(sq_gz/N - (double)calib.gz_off*calib.gz_off);

  Serial.println(F("  --- Ket qua hieu chinh Gyro ---"));
  Serial.print(F("  Offset Gx = ")); Serial.print(calib.gx_off,5); Serial.println(F(" deg/s"));
  Serial.print(F("  Offset Gy = ")); Serial.print(calib.gy_off,5); Serial.println(F(" deg/s"));
  Serial.print(F("  Offset Gz = ")); Serial.print(calib.gz_off,5); Serial.println(F(" deg/s"));
  Serial.print(F("  STD: Gx=")); Serial.print(std_gx,5);
  Serial.print(F(" Gy="));       Serial.print(std_gy,5);
  Serial.print(F(" Gz="));       Serial.println(std_gz,5);

  bool ok = (abs(calib.gx_off)<0.05f && abs(calib.gy_off)<0.05f && abs(calib.gz_off)<0.05f);
  Serial.println(ok ? F("  [OK] Gyro offset < 0.05 deg/s") : F("  [!] Mot so offset > 0.05 deg/s"));
  calib.done = true;
}

// ================================================================
//  BAI 2B: HIEU CHINH ACCEL 6-POSITION
// ================================================================
void task2_calib_accel_6pos() {
  Serial.println(F("\n[2B] HIEU CHINH ACCELEROMETER (6-position method)"));
  Serial.println(F("Chuan bi 6 vi tri cam bien, moi vi tri lay 200 mau."));
  Serial.println(F("Bam phim bat ky de chuyen sang vi tri tiep theo.\n"));

  const char* pos_labels[6] = {
    "Pos 1 - Z HUONG LEN   (mat cam bien nhin len tran)  -> Az ~ +1g",
    "Pos 2 - Z HUONG XUONG (lat nguoc, mat nhin xuong)   -> Az ~ -1g",
    "Pos 3 - X HUONG LEN   (dau truc X nhin len tran)    -> Ax ~ +1g",
    "Pos 4 - X HUONG XUONG (dau truc X nhin xuong dat)   -> Ax ~ -1g",
    "Pos 5 - Y HUONG LEN   (dau truc Y nhin len tran)    -> Ay ~ +1g",
    "Pos 6 - Y HUONG XUONG (dau truc Y nhin xuong dat)   -> Ay ~ -1g"
  };

  float means[6][3];
  const int N = 200;

  for (int pos = 0; pos < 6; pos++) {
    Serial.println();
    Serial.println(pos_labels[pos]);
    Serial.println(F(">> Dat dung vi tri roi nhan phim bat ky:"));
    wait_serial();
    delay(500);

    Serial.println(F("ax,ay,az"));

    double sax=0,say=0,saz=0;
    for (int i = 0; i < N; i++) {
      uint8_t buf[6];
      mpu_read_burst(REG_ACCEL_XOUT_H, buf, 6);
      float ax = (int16_t)((buf[0]<<8)|buf[1]) / ACCEL_SCALE_2G;
      float ay = (int16_t)((buf[2]<<8)|buf[3]) / ACCEL_SCALE_2G;
      float az = (int16_t)((buf[4]<<8)|buf[5]) / ACCEL_SCALE_2G;

      sax += ax; say += ay; saz += az;
      Serial.print(ax, 4); Serial.print(',');
      Serial.print(ay, 4); Serial.print(',');
      Serial.println(az, 4);
      delay(10);
    }
    means[pos][0] = sax/N;
    means[pos][1] = say/N;
    means[pos][2] = saz/N;
    Serial.print(F("  Mean -> Ax=")); Serial.print(means[pos][0],4);
    Serial.print(F("  Ay=")); Serial.print(means[pos][1],4);
    Serial.print(F("  Az=")); Serial.println(means[pos][2],4);
  }

  // offset = -(mean_positive + mean_negative) / 2
  calib.ax_off = -(means[2][0] + means[3][0]) / 2.0f;
  calib.ay_off = -(means[4][1] + means[5][1]) / 2.0f;
  calib.az_off = -(means[0][2] + means[1][2]) / 2.0f;
  calib.done   = true;

  Serial.println(F("\n--- Ket qua hieu chinh Accel ---"));
  Serial.print(F("  ax_offset = ")); Serial.println(calib.ax_off,5);
  Serial.print(F("  ay_offset = ")); Serial.println(calib.ay_off,5);
  Serial.print(F("  az_offset = ")); Serial.println(calib.az_off,5);

  float mag_z_up = sqrt(pow(means[0][0]+calib.ax_off,2)+pow(means[0][1]+calib.ay_off,2)+pow(means[0][2]+calib.az_off,2));
  float mag_x_up = sqrt(pow(means[2][0]+calib.ax_off,2)+pow(means[2][1]+calib.ay_off,2)+pow(means[2][2]+calib.az_off,2));
  Serial.print(F("  Kiem tra |a| sau calib (Pos1 - Z len): ")); Serial.print(mag_z_up,4); Serial.println(F(" g"));
  Serial.print(F("  Kiem tra |a| sau calib (Pos3 - X len): ")); Serial.print(mag_x_up,4); Serial.println(F(" g"));

  if (abs(mag_z_up-1.0f)<0.03f && abs(mag_x_up-1.0f)<0.03f)
    Serial.println(F("  [OK] |a| sau calib trong gioi han 3%."));
  else
    Serial.println(F("  [!] |a| sau calib lech >3%. Kiem tra lai vi tri dat cam bien."));
}

// ================================================================
//  BAI 2: HIEU CHINH TONG HOP
// ================================================================
void task2_calibrate() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  BAI 2: HIEU CHINH OFFSET (CALIBRATION)"));
  Serial.println(F("========================================"));
  Serial.println(F("Chon loai hieu chinh:"));
  Serial.println(F("  G - Chi hieu chinh Gyroscope (nam yen)"));
  Serial.println(F("  A - Chi hieu chinh Accelerometer (6-position)"));
  Serial.println(F("  B - Ca hai (Gyro + Accel)"));

  char c = toupper(wait_serial());

  if (c == 'G' || c == 'B') task2_calib_gyro();
  if (c == 'A' || c == 'B') task2_calib_accel_6pos();

  Serial.println(F("\n[OK] Hieu chinh hoan tat. Tat ca bai tiep theo se su dung offset nay."));
  Serial.println(F("Gui '0' de ve menu."));
}

// ================================================================
//  BAI 3: KHAO SAT ANH HUONG GIA TOC TRONG TRUONG
// ================================================================
void task3_gravity() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  BAI 3: ANH HUONG GIA TOC TRONG TRUONG"));
  Serial.println(F("========================================"));
  Serial.println(F("Quy tac: Ax = sin(pitch), Az = cos(pitch), Ay ~ 0 khi chi nghieng theo X"));
  Serial.println(F("\nBang ly thuyet (nghieng theo truc X - Pitch):"));
  Serial.println(F(" Goc |  Ax(g) |  Ay(g) |  Az(g) | |a|(g)"));
  Serial.println(F("-----|--------|--------|--------|-------"));

  float theory[][4] = {
    {0,    0.000f, 0, 1.000f},
    {15,   0.259f, 0, 0.966f},
    {30,   0.500f, 0, 0.866f},
    {45,   0.707f, 0, 0.707f},
    {60,   0.866f, 0, 0.500f},
    {90,   1.000f, 0, 0.000f}
  };
  for (int i = 0; i < 6; i++) {
    Serial.print(F("  ")); Serial.print((int)theory[i][0], DEC);
    Serial.print(F("  | ")); Serial.print(theory[i][1], 3);
    Serial.print(F("  | ")); Serial.print(theory[i][2], 3);
    Serial.print(F("  | ")); Serial.print(theory[i][3], 3);
    Serial.println(F("  | 1.000"));
  }

  Serial.println(F("\n--- DO THUC TE ---"));
  Serial.println(F("Goc | Ax_do | Ay_do | Az_do | |a| | Sai lech%"));

  float angles[] = {0, 15, 30, 45, 60, 90};

  for (int i = 0; i < 6; i++) {
    Serial.print(F("\nDat cam bien nghieng "));
    Serial.print((int)angles[i]);
    Serial.println(F(" do (theo truc X), nhan phim:"));
    wait_serial();
    delay(500);

    Serial.println(F("angle_deg,ax,ay,az"));

    double sax = 0, say = 0, saz = 0;
    for (int j = 0; j < 200; j++) {
      read_sensor();
      Serial.print(angles[i], 1); Serial.print(",");
      Serial.print(imu.ax_g, 4); Serial.print(",");
      Serial.print(imu.ay_g, 4); Serial.print(",");
      Serial.println(imu.az_g, 4);
      sax += imu.ax_g;
      say += imu.ay_g;
      saz += imu.az_g;
      delay(10);
    }

    float ax  = sax / 200;
    float ay  = say / 200;
    float az  = saz / 200;
    float mag = sqrt(ax*ax + ay*ay + az*az);
    float err = abs(mag - 1.0f) * 100.0f;

    Serial.print(F("  ")); Serial.print((int)angles[i]);
    Serial.print(F("  | ")); Serial.print(ax, 3);
    Serial.print(F("  | ")); Serial.print(ay, 3);
    Serial.print(F("  | ")); Serial.print(az, 3);
    Serial.print(F("  | ")); Serial.print(mag, 4);
    Serial.print(F("  | ")); Serial.print(err, 2); Serial.println(F("%"));

    if (err > 3.0f) Serial.println(F("  [!] Sai lech > 3%! Can hieu chinh gain."));
    else            Serial.println(F("  [OK]"));
  }

  Serial.println(F("\nBai 3 hoan tat. Gui '0' de ve menu."));
}

// ================================================================
//  BAI 4: BO LOC TIN HIEU
// ================================================================
void task4_filter() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  BAI 4: BO LOC TIN HIEU"));
  Serial.println(F("========================================"));
  Serial.println(F("Chon che do:"));
  Serial.println(F("  C - Complementary Filter"));
  Serial.println(F("  K - Kalman Filter"));
  Serial.println(F("  B - Ca hai (so sanh song song)"));
  Serial.println(F("  A - Dat alpha tuy chinh (cho Complementary)"));

  char mode = toupper(wait_serial());

  if (mode == 'A') {
    Serial.println(F("Nhap alpha (vi du: 0.98, Enter de xac nhan):"));
    while (!Serial.available()) delay(50);
    cf_alpha = Serial.parseFloat();
    flush_serial();
    if (cf_alpha < 0.8f || cf_alpha > 0.9999f) cf_alpha = 0.98f;
    Serial.print(F("  Alpha = ")); Serial.println(cf_alpha, 3);
    mode = 'C';
  } else if (mode == 'C') {
    Serial.print(F("  Dung alpha mac dinh = ")); Serial.println(cf_alpha);
  }

  // Reset bo loc
  cf_roll = cf_pitch = 0;
  kf_roll   = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  kf_pitch  = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  last_us = micros();

  // Khoi tao goc tu accel
  read_sensor();
  cf_roll     = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
  cf_pitch    = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;
  kf_roll.angle  = cf_roll;
  kf_pitch.angle = cf_pitch;

  Serial.println(F("\nXuat CSV (Serial Plotter):"));
  Serial.println(F("Time_ms,Roll_Accel,Pitch_Accel,Roll_CF,Pitch_CF,Roll_KF,Pitch_KF"));
  Serial.println(F("Gui 'q' de dung..."));
  delay(500);

  while (true) {
    if (Serial.available()) { char c = Serial.read(); if (c=='q'||c=='Q') break; }

    unsigned long now = micros();
    dt = (now - last_us) / 1000000.0f;
    last_us = now;
    if (dt <= 0 || dt > 0.5f) dt = 0.01f;

    read_sensor();

    float roll_a  = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
    float pitch_a = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;

    // Complementary Filter
    cf_roll  = cf_alpha*(cf_roll  + imu.gx_dps*dt) + (1.0f-cf_alpha)*roll_a;
    cf_pitch = cf_alpha*(cf_pitch + imu.gy_dps*dt) + (1.0f-cf_alpha)*pitch_a;

    // Kalman Filter
    kf_roll_out  = kalman_update(&kf_roll,  roll_a,  imu.gx_dps, dt);
    kf_pitch_out = kalman_update(&kf_pitch, pitch_a, imu.gy_dps, dt);

    Serial.print(millis());    Serial.print(',');
    Serial.print(roll_a,2);    Serial.print(',');
    Serial.print(pitch_a,2);   Serial.print(',');
    if (mode=='C'||mode=='B') {
      Serial.print(cf_roll,2); Serial.print(','); Serial.print(cf_pitch,2);
    } else {
      Serial.print(F("0,0"));
    }
    Serial.print(',');
    if (mode=='K'||mode=='B') {
      Serial.print(kf_roll_out,2); Serial.print(','); Serial.print(kf_pitch_out,2);
    } else {
      Serial.print(F("0,0"));
    }
    Serial.println();
    delay(10);
  }

  Serial.println(F("\nBai 4 dung. Gui '0' de ve menu."));
}

// ================================================================
//  BAI 5: DO SHOCK & PHAN TICH DAO DONG
// ================================================================
void task5_shock() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  BAI 5: DO SHOCK & PHAN TICH DAO DONG"));
  Serial.println(F("========================================"));
  Serial.println(F("Chon che do:"));
  Serial.println(F("  S - Shock detection (phat hien va cham real-time)"));
  Serial.println(F("  V - Vibration sampling 512 mau @ 500Hz (cho MATLAB FFT)"));

  char mode = toupper(wait_serial());

  if (mode == 'S') {
    Serial.println(F("\n[SHOCK DETECTION]"));
    Serial.print(F("Nguong: |a| - 1g > ")); Serial.print(SHOCK_THRESHOLD); Serial.println(F(" g"));
    Serial.println(F("Xuat: Time_ms,Ax,Ay,Az,Magnitude,SHOCK(0/1)"));
    Serial.println(F("Gui 'q' de dung.\n"));
    delay(500);

    bool in_shock     = false;
    unsigned long t_s = 0;
    float peak_g      = 0;

    mpu_write(REG_SMPLRT_DIV, 1);   // 500 Hz
    delay(50);

    while (true) {
      if (Serial.available()) { char c=Serial.read(); if(c=='q'||c=='Q') break; }

      unsigned long t0 = micros();

      read_sensor();
      float mag = sqrt(imu.ax_g*imu.ax_g + imu.ay_g*imu.ay_g + imu.az_g*imu.az_g);
      bool is_shock = (abs(mag - 1.0f) > SHOCK_THRESHOLD);

      if (is_shock && !in_shock) {
        in_shock = true;
        t_s = millis();
        peak_g = mag;
        Serial.println(F(">>> SHOCK DETECTED! <<<"));
      }
      if (is_shock && in_shock && mag > peak_g) peak_g = mag;
      if (!is_shock && in_shock) {
        unsigned long dur = millis() - t_s;
        Serial.print(F(">>> SHOCK KET THUC. Thoi gian: "));
        Serial.print(dur);
        Serial.print(F(" ms | Bien do dinh: "));
        Serial.print(peak_g,3);
        Serial.println(F(" g <<<"));
        in_shock = false;
        peak_g = 0;
      }

      Serial.print(millis()); Serial.print(',');
      Serial.print(imu.ax_g,3); Serial.print(',');
      Serial.print(imu.ay_g,3); Serial.print(',');
      Serial.print(imu.az_g,3); Serial.print(',');
      Serial.print(mag,3);      Serial.print(',');
      Serial.println(is_shock ? 1 : 0);

      while ((micros() - t0) < 2000UL);
    }

    mpu_write(REG_SMPLRT_DIV, 9);   // Tra ve 100 Hz

  } else {
    Serial.println(F("\n[VIBRATION SAMPLING]"));
    Serial.println(F("Se lay 512 mau gia toc (|a|-1g) tai 500Hz."));
    Serial.println(F("Nhan phim bat ky de bat dau lay mau:"));
    wait_serial();

    mpu_write(REG_SMPLRT_DIV, 1);   // 500 Hz
    delay(100);

    Serial.println(F("SAMPLE_START"));
    Serial.println(F("fs=500"));
    Serial.println(F("n=512"));

    for (int i = 0; i < 512; i++) {
      unsigned long t0 = micros();

      uint8_t buf[6];
      mpu_read_burst(REG_ACCEL_XOUT_H, buf, 6);
      float ax = (int16_t)((buf[0]<<8)|buf[1]) / ACCEL_SCALE_2G;
      float ay = (int16_t)((buf[2]<<8)|buf[3]) / ACCEL_SCALE_2G;
      float az = (int16_t)((buf[4]<<8)|buf[5]) / ACCEL_SCALE_2G;

      if (calib.done) { ax += calib.ax_off; ay += calib.ay_off; az += calib.az_off; }

      float mag = sqrt(ax*ax+ay*ay+az*az) - 1.0f;
      Serial.println(mag, 5);

      while ((micros() - t0) < 2000UL);
    }

    Serial.println(F("SAMPLE_END"));
    Serial.println(F("\n[OK] Xuat 512 mau xong!"));
    Serial.println(F("  1. Copy du lieu tu 'SAMPLE_START' den 'SAMPLE_END'"));
    Serial.println(F("  2. Luu vao file bai5_fft_data.txt"));
    Serial.println(F("  3. Chay phan_5.m trong MATLAB"));

    mpu_write(REG_SMPLRT_DIV, 9);   // Tra ve 100 Hz
  }

  Serial.println(F("\nBai 5 dung. Gui '0' de ve menu."));
}

// ================================================================
//  BAI 6: UOC LUONG GOC ROLL / PITCH / YAW
// ================================================================
void task6_angles() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  BAI 6: UOC LUONG GOC ROLL / PITCH / YAW"));
  Serial.println(F("========================================"));
  Serial.println(F("Phuong phap:"));
  Serial.println(F("  Roll, Pitch: Kalman Filter (Accel + Gyro)"));
  Serial.println(F("  Yaw        : Tich phan Gyro (drift theo thoi gian)"));
  Serial.println(F("\nXuat CSV:"));
  Serial.println(F("Time_ms,Roll_Accel,Pitch_Accel,Roll_Gyro,Pitch_Gyro,Yaw_Gyro,Roll_KF,Pitch_KF,Yaw_KF"));
  Serial.println(F("Gui 'q' de dung.\n"));
  delay(500);

  float roll_g=0, pitch_g=0, yaw_g=0, yaw_kf=0;
  kf_roll  = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  kf_pitch = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  last_us  = micros();

  read_sensor();
  float r0 = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
  float p0 = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;
  roll_g = r0; pitch_g = p0;
  kf_roll.angle = r0; kf_pitch.angle = p0;

  while (true) {
    if (Serial.available()) { char c=Serial.read(); if(c=='q'||c=='Q') break; }

    unsigned long now = micros();
    dt = (now - last_us) / 1000000.0f;
    last_us = now;
    if (dt<=0||dt>0.5f) dt=0.01f;

    read_sensor();

    float roll_a  = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
    float pitch_a = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;

    roll_g  += imu.gx_dps * dt;
    pitch_g += imu.gy_dps * dt;
    yaw_g   += imu.gz_dps * dt;
    yaw_kf  += imu.gz_dps * dt;

    kf_roll_out  = kalman_update(&kf_roll,  roll_a,  imu.gx_dps, dt);
    kf_pitch_out = kalman_update(&kf_pitch, pitch_a, imu.gy_dps, dt);

    Serial.print(millis());      Serial.print(',');
    Serial.print(roll_a,2);      Serial.print(',');
    Serial.print(pitch_a,2);     Serial.print(',');
    Serial.print(roll_g,2);      Serial.print(',');
    Serial.print(pitch_g,2);     Serial.print(',');
    Serial.print(yaw_g,2);       Serial.print(',');
    Serial.print(kf_roll_out,2); Serial.print(',');
    Serial.print(kf_pitch_out,2);Serial.print(',');
    Serial.println(yaw_kf,2);

    delay(10);
  }

  Serial.println(F("\nBai 6 dung. Gui '0' de ve menu."));
}

// ================================================================
//  BAI 7: BAI TAP TONG HOP
// ================================================================
void task7_summary() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  BAI 7: BAI TAP TONG HOP"));
  Serial.println(F("========================================"));
  Serial.println(F("He thong giam sat IMU day du:"));
  Serial.println(F("  [1] Tu dong hieu chinh Gyro khi khoi dong (5s)"));
  Serial.println(F("  [2] Kalman Filter cho Roll & Pitch"));
  Serial.println(F("  [3] Phat hien Shock tu dong (nguong: 2g)"));
  Serial.println(F("  [4] Giam sat nhiet do cam bien"));
  Serial.println(F("  [5] Xuat CSV cho Serial Plotter"));
  Serial.println(F("\nXuat: Time_ms,Roll_KF,Pitch_KF,Yaw_Gyro,Temp_C,Accel_Mag,SHOCK"));
  Serial.println(F("Gui 'q' de dung.\n"));

  Serial.println(F("[1/2] HIEU CHINH GYRO TU DONG..."));
  Serial.println(F("      Giu THIET BI YEN LANG trong 5 giay!"));
  delay(2000);

  bool stable   = false;
  int  attempts = 0;

  while (!stable && attempts < 5) {
    double sgx=0,sgy=0,sgz=0;
    double sq_gx=0,sq_gy=0,sq_gz=0;
    for (int i=0; i<500; i++) {
      uint8_t buf[14];
      mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);
      float gx = (int16_t)((buf[8]<<8)|buf[9])   / GYRO_SCALE_250;
      float gy = (int16_t)((buf[10]<<8)|buf[11])  / GYRO_SCALE_250;
      float gz = (int16_t)((buf[12]<<8)|buf[13])  / GYRO_SCALE_250;
      sgx+=gx; sgy+=gy; sgz+=gz;
      sq_gx+=gx*gx; sq_gy+=gy*gy; sq_gz+=gz*gz;
      delay(10);
    }
    float mgx = sgx/500.0f, mgy = sgy/500.0f, mgz = sgz/500.0f;
    float std_total = sqrt(sq_gx/500-(double)mgx*mgx)
                    + sqrt(sq_gy/500-(double)mgy*mgy)
                    + sqrt(sq_gz/500-(double)mgz*mgz);

    if (std_total < 0.5f) {
      calib.gx_off = mgx; calib.gy_off = mgy; calib.gz_off = mgz;
      calib.done   = true;
      stable = true;
      Serial.print(F("  [OK] Gyro offset: Gx=")); Serial.print(mgx,4);
      Serial.print(F(" Gy=")); Serial.print(mgy,4);
      Serial.print(F(" Gz=")); Serial.println(mgz,4);
    } else {
      attempts++;
      Serial.print(F("  [!] Khong on dinh (STD=")); Serial.print(std_total,3);
      Serial.print(F("). Thu lai lan ")); Serial.println(attempts);
      if (attempts < 5) {
        Serial.println(F("      Giu yen thiet bi them 5 giay..."));
        delay(2000);
      }
    }
  }

  if (!stable) {
    Serial.println(F("  [WARN] Khong the on dinh sau 5 lan thu. Su dung offset cuoi cung."));
    calib.done = true;
  }

  Serial.println(F("[2/2] Khoi tao goc ban dau tu Accel..."));
  read_sensor();
  float roll  = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
  float pitch = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;
  float yaw   = 0;

  kf_roll  = {roll,  0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  kf_pitch = {pitch, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  last_us  = micros();

  Serial.println(F("\nBat dau giam sat. Gui 'q' de dung."));
  Serial.println(F("Time_ms,Roll_KF,Pitch_KF,Yaw_Gyro,Temp_C,Accel_Mag,SHOCK"));

  bool in_shock     = false;
  unsigned long shock_t = 0;

  while (true) {
    if (Serial.available()) { char c=Serial.read(); if(c=='q'||c=='Q') break; }

    unsigned long now = micros();
    dt = (now - last_us) / 1000000.0f;
    last_us = now;
    if (dt<=0||dt>0.5f) dt=0.01f;

    read_sensor();

    float roll_a  = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
    float pitch_a = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;

    roll  = kalman_update(&kf_roll,  roll_a,  imu.gx_dps, dt);
    pitch = kalman_update(&kf_pitch, pitch_a, imu.gy_dps, dt);
    yaw  += imu.gz_dps * dt;

    float mag   = sqrt(imu.ax_g*imu.ax_g+imu.ay_g*imu.ay_g+imu.az_g*imu.az_g);
    bool  shock = (abs(mag-1.0f) > SHOCK_THRESHOLD);

    if (shock && !in_shock) {
      in_shock = true; shock_t = millis();
      Serial.println(F("# >>> SHOCK DETECTED! <<<"));
    }
    if (!shock && in_shock) {
      in_shock = false;
      Serial.print(F("# >>> SHOCK END. Duration: "));
      Serial.print(millis()-shock_t); Serial.println(F(" ms"));
    }

    Serial.print(millis());     Serial.print(',');
    Serial.print(roll,2);       Serial.print(',');
    Serial.print(pitch,2);      Serial.print(',');
    Serial.print(yaw,2);        Serial.print(',');
    Serial.print(imu.temp_c,1); Serial.print(',');
    Serial.print(mag,3);        Serial.print(',');
    Serial.println(shock ? 1 : 0);

    delay(10);
  }

  Serial.println(F("\nBai 7 dung. Gui '0' de ve menu."));
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);   // I2C Fast Mode 400kHz
  delay(500);

  Serial.println(F("\n=== MPU6050 LAB - KY THUAT CAM BIEN ==="));
  mpu_setup();
  print_menu();
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    flush_serial();

    switch (c) {
      case '1': task1_static();    print_menu(); break;
      case '2': task2_calibrate(); print_menu(); break;
      case '3': task3_gravity();   print_menu(); break;
      case '4': task4_filter();    print_menu(); break;
      case '5': task5_shock();     print_menu(); break;
      case '6': task6_angles();    print_menu(); break;
      case '7': task7_summary();   print_menu(); break;
      case '0': print_menu();      break;
      default:  break;
    }
  }
}
