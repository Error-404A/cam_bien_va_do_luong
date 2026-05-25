#include <Wire.h>
#include <math.h>

#define MPU_ADDR         0x68
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_ENABLE   0x38
#define REG_INT_STATUS   0x3A
#define REG_ACCEL_XOUT_H 0x3B
#define REG_TEMP_OUT_H   0x41
#define REG_GYRO_XOUT_H  0x43
#define REG_PWR_MGMT_1   0x6B
#define REG_PWR_MGMT_2   0x6C
#define REG_WHO_AM_I     0x75

#define ACCEL_SCALE_2G   16384.0f
#define ACCEL_SCALE_4G    8192.0f
#define ACCEL_SCALE_8G    4096.0f
#define GYRO_SCALE_250     131.0f
#define GYRO_SCALE_500      65.5f
#define GYRO_SCALE_1000     32.8f
#define RAD_TO_DEG        57.2957795f
#define DEG_TO_RAD         0.0174533f
#define SHOCK_THRESHOLD    2.0f

struct SensorData {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  int16_t temp_raw;
  float ax_g,  ay_g,  az_g;
  float gx_dps, gy_dps, gz_dps;
  float temp_c;
};

struct CalibData {
  float ax_off, ay_off, az_off;
  float gx_off, gy_off, gz_off;
  bool done;
};

struct KalmanFilter {
  float angle;
  float bias;
  float P[2][2];
  float Q_angle;
  float Q_bias;
  float R_measure;
};

SensorData  imu;
CalibData   calib = {0,0,0, 0,0,0, false};
KalmanFilter kf_roll  = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
KalmanFilter kf_pitch = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};

float cf_alpha    = 0.98f;
float cf_roll     = 0.0f;
float cf_pitch    = 0.0f;
float kf_roll_out  = 0.0f;
float kf_pitch_out = 0.0f;

unsigned long last_us = 0;
float dt = 0.01f;

void mpu_write(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t mpu_read_byte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0;
}

void mpu_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, len, (uint8_t)true);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = Wire.available() ? Wire.read() : 0;
}

void mpu_setup() {
  uint8_t who = mpu_read_byte(REG_WHO_AM_I);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
  if      (who == 0x68) Serial.println(F("MPU6050 OK"));
  else if (who == 0x70) Serial.println(F("WHO_AM_I=0x70: MPU6500"));
  else if (who == 0x71) Serial.println(F("WHO_AM_I=0x71: MPU9250"));
  else                  Serial.println(F("Khong nhan dien duoc, kiem tra I2C"));

  mpu_write(REG_PWR_MGMT_1, 0x80);
  delay(150);
  mpu_write(REG_PWR_MGMT_1, 0x01);
  delay(50);
  mpu_write(REG_CONFIG,      0x03);
  mpu_write(REG_GYRO_CONFIG, 0x00);
  mpu_write(REG_ACCEL_CONFIG,0x00);
  mpu_write(REG_SMPLRT_DIV,  9);
  Serial.println(F("+-2g / +-250dps / 100Hz / DLPF 44Hz"));
}

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

  imu.ax_g   = imu.ax_raw / ACCEL_SCALE_2G;
  imu.ay_g   = imu.ay_raw / ACCEL_SCALE_2G;
  imu.az_g   = imu.az_raw / ACCEL_SCALE_2G;
  imu.gx_dps = imu.gx_raw / GYRO_SCALE_250;
  imu.gy_dps = imu.gy_raw / GYRO_SCALE_250;
  imu.gz_dps = imu.gz_raw / GYRO_SCALE_250;
  imu.temp_c = (imu.temp_raw / 340.0f) + 36.53f;

  if (calib.done) {
    imu.ax_g   += calib.ax_off;
    imu.ay_g   += calib.ay_off;
    imu.az_g   += calib.az_off;
    imu.gx_dps -= calib.gx_off;
    imu.gy_dps -= calib.gy_off;
    imu.gz_dps -= calib.gz_off;
  }
}

float kalman_update(KalmanFilter* kf, float accel_angle, float gyro_rate, float dt_s) {
  float rate = gyro_rate - kf->bias;
  kf->angle += dt_s * rate;

  kf->P[0][0] += dt_s * (dt_s * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
  kf->P[0][1] -= dt_s * kf->P[1][1];
  kf->P[1][0] -= dt_s * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt_s;

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

void print_menu() {
  Serial.println(F("--- MPU6050 ---"));
  Serial.println(F("1 - Do luong co ban"));
  Serial.println(F("2 - Calibration"));
  Serial.println(F("3 - Anh huong gia toc trong truong"));
  Serial.println(F("4 - Bo loc (Complementary & Kalman)"));
  Serial.println(F("5 - Shock & FFT sampling"));
  Serial.println(F("6 - Roll/Pitch/Yaw"));
  Serial.println(F("7 - Tong hop"));
  Serial.println(F("0 - Menu"));
  Serial.print(F("Calib: "));
  Serial.println(calib.done ? F("DA calib") : F("CHUA calib"));
}

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

void task1_static() {
  Serial.println(F("BAI 1: DO LUONG CO BAN"));
  Serial.println(F("Dat cam bien nam yen, chuan bi lay 500 mau..."));
  delay(3000);

  const int N = 500;
  double sum_ax=0, sum_ay=0, sum_az=0;
  double sum_gx=0, sum_gy=0, sum_gz=0;
  double sq_ax=0,  sq_ay=0,  sq_az=0;
  double sq_gx=0,  sq_gy=0,  sq_gz=0;

  Serial.println(F("ax,ay,az,gx,gy,gz"));

  for (int i = 0; i < N; i++) {
    uint8_t buf[14];
    mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);

    float ax = (int16_t)((buf[0]<<8)|buf[1])   / ACCEL_SCALE_2G;
    float ay = (int16_t)((buf[2]<<8)|buf[3])   / ACCEL_SCALE_2G;
    float az = (int16_t)((buf[4]<<8)|buf[5])   / ACCEL_SCALE_2G;
    float gx = (int16_t)((buf[8]<<8)|buf[9])   / GYRO_SCALE_250;
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
  Serial.println(F("[ACCEL] (g)"));
  Serial.print(F("  Mean: Ax=")); Serial.print(m_ax,5);
  Serial.print(F("  Ay="));      Serial.print(m_ay,5);
  Serial.print(F("  Az="));      Serial.println(m_az,5);
  Serial.print(F("  STD:  Ax=")); Serial.print(s_ax,6);
  Serial.print(F("  Ay="));      Serial.print(s_ay,6);
  Serial.print(F("  Az="));      Serial.println(s_az,6);

  Serial.println(F("[GYRO] (deg/s)"));
  Serial.print(F("  Mean: Gx=")); Serial.print(m_gx,5);
  Serial.print(F("  Gy="));       Serial.print(m_gy,5);
  Serial.print(F("  Gz="));       Serial.println(m_gz,5);
  Serial.print(F("  STD:  Gx=")); Serial.print(s_gx,6);
  Serial.print(F("  Gy="));       Serial.print(s_gy,6);
  Serial.print(F("  Gz="));       Serial.println(s_gz,6);

  Serial.print(F("|a| = ")); Serial.print(mag,5);
  Serial.print(F(" g  Sai lech: ")); Serial.print(abs(mag-1.0f)*100,2); Serial.println(F("%"));

  if (abs(m_ax)<0.05f && abs(m_ay)<0.05f) Serial.println(F("Accel X,Y offset OK"));
  else                                     Serial.println(F("Accel offset lon, can calib"));

  if (abs(m_gx)<0.5f && abs(m_gy)<0.5f && abs(m_gz)<0.5f) Serial.println(F("Gyro offset OK"));
  else                                                      Serial.println(F("Gyro offset lon, can calib"));

  if (abs(mag-1.0f)<0.03f) Serial.println(F("|a| trong gioi han 3%"));
  else                     Serial.println(F("|a| lech >3%"));

  Serial.println(F("\nBai 1 xong. Gui '0' de ve menu."));
}

void task2_calib_gyro() {
  Serial.println(F("\n[2A] CALIB GYRO - Giu cam bien yen, nhan phim:"));
  wait_serial();

  const int N = 500;
  double sgx=0, sgy=0, sgz=0;
  double sq_gx=0, sq_gy=0, sq_gz=0;

  Serial.println(F("gx,gy,gz"));

  for (int i = 0; i < N; i++) {
    uint8_t buf[14];
    mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);
    float gx = (int16_t)((buf[8]<<8)|buf[9])   / GYRO_SCALE_250;
    float gy = (int16_t)((buf[10]<<8)|buf[11]) / GYRO_SCALE_250;
    float gz = (int16_t)((buf[12]<<8)|buf[13]) / GYRO_SCALE_250;
    sgx+=gx; sgy+=gy; sgz+=gz;
    sq_gx+=gx*gx; sq_gy+=gy*gy; sq_gz+=gz*gz;
    Serial.print(gx,4); Serial.print(',');
    Serial.print(gy,4); Serial.print(',');
    Serial.println(gz,4);
    delay(10);
  }

  calib.gx_off = sgx/N;
  calib.gy_off = sgy/N;
  calib.gz_off = sgz/N;

  float std_gx = sqrt(sq_gx/N - (double)calib.gx_off*calib.gx_off);
  float std_gy = sqrt(sq_gy/N - (double)calib.gy_off*calib.gy_off);
  float std_gz = sqrt(sq_gz/N - (double)calib.gz_off*calib.gz_off);

  Serial.print(F("Offset Gx=")); Serial.print(calib.gx_off,5);
  Serial.print(F(" Gy="));       Serial.print(calib.gy_off,5);
  Serial.print(F(" Gz="));       Serial.println(calib.gz_off,5);
  Serial.print(F("STD Gx="));    Serial.print(std_gx,5);
  Serial.print(F(" Gy="));       Serial.print(std_gy,5);
  Serial.print(F(" Gz="));       Serial.println(std_gz,5);

  calib.done = true;
}

void task2_calib_accel_6pos() {
  Serial.println(F("\n[2B] CALIB ACCEL 6 vi tri"));
  const char* pos_labels[6] = {
    "Pos1 - Z len  (Az~+1g)",
    "Pos2 - Z xuong (Az~-1g)",
    "Pos3 - X len  (Ax~+1g)",
    "Pos4 - X xuong (Ax~-1g)",
    "Pos5 - Y len  (Ay~+1g)",
    "Pos6 - Y xuong (Ay~-1g)"
  };

  float means[6][3];
  const int N = 200;

  for (int pos = 0; pos < 6; pos++) {
    Serial.println(pos_labels[pos]);
    Serial.println(F("Dat dung roi nhan phim:"));
    wait_serial();
    delay(500);
    Serial.println(F("ax,ay,az"));

    double sax=0, say=0, saz=0;
    for (int i = 0; i < N; i++) {
      uint8_t buf[6];
      mpu_read_burst(REG_ACCEL_XOUT_H, buf, 6);
      float ax = (int16_t)((buf[0]<<8)|buf[1]) / ACCEL_SCALE_2G;
      float ay = (int16_t)((buf[2]<<8)|buf[3]) / ACCEL_SCALE_2G;
      float az = (int16_t)((buf[4]<<8)|buf[5]) / ACCEL_SCALE_2G;
      sax+=ax; say+=ay; saz+=az;
      Serial.print(ax,4); Serial.print(',');
      Serial.print(ay,4); Serial.print(',');
      Serial.println(az,4);
      delay(10);
    }
    means[pos][0] = sax/N;
    means[pos][1] = say/N;
    means[pos][2] = saz/N;
    Serial.print(F("Mean Ax=")); Serial.print(means[pos][0],4);
    Serial.print(F(" Ay="));     Serial.print(means[pos][1],4);
    Serial.print(F(" Az="));     Serial.println(means[pos][2],4);
  }

  calib.ax_off = -(means[2][0] + means[3][0]) / 2.0f;
  calib.ay_off = -(means[4][1] + means[5][1]) / 2.0f;
  calib.az_off = -(means[0][2] + means[1][2]) / 2.0f;
  calib.done   = true;

  Serial.print(F("ax_off=")); Serial.println(calib.ax_off,5);
  Serial.print(F("ay_off=")); Serial.println(calib.ay_off,5);
  Serial.print(F("az_off=")); Serial.println(calib.az_off,5);

  float mag_z_up = sqrt(pow(means[0][0]+calib.ax_off,2)+pow(means[0][1]+calib.ay_off,2)+pow(means[0][2]+calib.az_off,2));
  float mag_x_up = sqrt(pow(means[2][0]+calib.ax_off,2)+pow(means[2][1]+calib.ay_off,2)+pow(means[2][2]+calib.az_off,2));
  Serial.print(F("|a| Pos1=")); Serial.print(mag_z_up,4);
  Serial.print(F("  Pos3=")); Serial.println(mag_x_up,4);
}

void task2_calibrate() {
  Serial.println(F("BAI 2: CALIBRATION"));
  Serial.println(F("G - Gyro  /  A - Accel  /  B - Ca hai"));

  char c = toupper(wait_serial());
  if (c=='G' || c=='B') task2_calib_gyro();
  if (c=='A' || c=='B') task2_calib_accel_6pos();

  Serial.println(F("Calib xong. Gui '0' de ve menu."));
}

void task3_gravity() {
  Serial.println(F("BAI 3: ANH HUONG GIA TOC TRONG TRUONG"));
  Serial.println(F("Ly thuyet (nghieng theo X):"));
  Serial.println(F(" Goc | Ax    | Ay | Az    | |a|"));
  float theory[][4] = {
    {0,   0.000f, 0, 1.000f},
    {15,  0.259f, 0, 0.966f},
    {30,  0.500f, 0, 0.866f},
    {45,  0.707f, 0, 0.707f},
    {60,  0.866f, 0, 0.500f},
    {90,  1.000f, 0, 0.000f}
  };
  for (int i = 0; i < 6; i++) {
    Serial.print(F("  ")); Serial.print((int)theory[i][0]);
    Serial.print(F(" | "));  Serial.print(theory[i][1],3);
    Serial.print(F(" | "));  Serial.print(theory[i][2],3);
    Serial.print(F(" | "));  Serial.print(theory[i][3],3);
    Serial.println(F(" | 1.000"));
  }

  Serial.println(F("\n--- DO THUC TE ---"));
  Serial.println(F("angle_deg,ax,ay,az"));

  float angles[] = {0, 15, 30, 45, 60, 90};
  for (int i = 0; i < 6; i++) {
    Serial.print(F("Nghieng ")); Serial.print((int)angles[i]); Serial.println(F(" do, nhan phim:"));
    wait_serial();
    delay(500);

    double sax=0, say=0, saz=0;
    for (int j = 0; j < 200; j++) {
      read_sensor();
      Serial.print(angles[i],1); Serial.print(',');
      Serial.print(imu.ax_g,4);  Serial.print(',');
      Serial.print(imu.ay_g,4);  Serial.print(',');
      Serial.println(imu.az_g,4);
      sax+=imu.ax_g; say+=imu.ay_g; saz+=imu.az_g;
      delay(10);
    }

    float ax  = sax/200, ay = say/200, az = saz/200;
    float mag = sqrt(ax*ax + ay*ay + az*az);
    float err = abs(mag-1.0f)*100.0f;

    Serial.print(F("  ")); Serial.print((int)angles[i]);
    Serial.print(F(" | ")); Serial.print(ax,3);
    Serial.print(F(" | ")); Serial.print(ay,3);
    Serial.print(F(" | ")); Serial.print(az,3);
    Serial.print(F(" | ")); Serial.print(mag,4);
    Serial.print(F(" | ")); Serial.print(err,2); Serial.println(F("%"));
  }

  Serial.println(F("\nBai 3 xong. Gui '0' de ve menu."));
}

void task4_filter() {
  Serial.println(F("BAI 4: BO LOC"));
  Serial.println(F("C - Complementary  /  K - Kalman  /  B - Ca hai  /  A - Tuy chinh alpha"));

  char mode = toupper(wait_serial());

  if (mode == 'A') {
    Serial.println(F("Nhap alpha (0.80-0.9999):"));
    while (!Serial.available()) delay(50);
    cf_alpha = Serial.parseFloat();
    flush_serial();
    if (cf_alpha < 0.8f || cf_alpha > 0.9999f) cf_alpha = 0.98f;
    Serial.print(F("Alpha = ")); Serial.println(cf_alpha,3);
    mode = 'C';
  }

  cf_roll = cf_pitch = 0;
  kf_roll  = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  kf_pitch = {0, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  last_us  = micros();

  read_sensor();
  cf_roll     = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
  cf_pitch    = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;
  kf_roll.angle  = cf_roll;
  kf_pitch.angle = cf_pitch;

  Serial.println(F("Time_ms,Roll_Accel,Pitch_Accel,Roll_CF,Pitch_CF,Roll_KF,Pitch_KF"));
  Serial.println(F("Gui 'q' de dung"));
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

    cf_roll  = cf_alpha*(cf_roll  + imu.gx_dps*dt) + (1.0f-cf_alpha)*roll_a;
    cf_pitch = cf_alpha*(cf_pitch + imu.gy_dps*dt) + (1.0f-cf_alpha)*pitch_a;

    kf_roll_out  = kalman_update(&kf_roll,  roll_a,  imu.gx_dps, dt);
    kf_pitch_out = kalman_update(&kf_pitch, pitch_a, imu.gy_dps, dt);

    Serial.print(millis());  Serial.print(',');
    Serial.print(roll_a,2);  Serial.print(',');
    Serial.print(pitch_a,2); Serial.print(',');
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

  Serial.println(F("\nBai 4 xong. Gui '0' de ve menu."));
}

void task5_shock() {
  Serial.println(F("BAI 5: SHOCK & DAO DONG"));
  Serial.println(F("S - Shock detection  /  V - Vibration sampling 512 mau @ 500Hz"));

  char mode = toupper(wait_serial());

  if (mode == 'S') {
    Serial.println(F("Time_ms,Ax,Ay,Az,Magnitude,SHOCK"));
    Serial.println(F("Gui 'q' de dung"));
    delay(500);

    bool in_shock = false;
    unsigned long t_s = 0;
    float peak_g = 0;

    mpu_write(REG_SMPLRT_DIV, 1);
    delay(50);

    while (true) {
      if (Serial.available()) { char c=Serial.read(); if(c=='q'||c=='Q') break; }

      unsigned long t0 = micros();
      read_sensor();

      float mag = sqrt(imu.ax_g*imu.ax_g + imu.ay_g*imu.ay_g + imu.az_g*imu.az_g);
      bool is_shock = (abs(mag-1.0f) > SHOCK_THRESHOLD);

      if (is_shock && !in_shock) {
        in_shock = true;
        t_s = millis();
        peak_g = mag;
        Serial.println(F(">>> SHOCK DETECTED!"));
      }
      if (is_shock && in_shock && mag > peak_g) peak_g = mag;
      if (!is_shock && in_shock) {
        Serial.print(F(">>> SHOCK END. "));
        Serial.print(millis()-t_s); Serial.print(F(" ms | peak: "));
        Serial.print(peak_g,3); Serial.println(F(" g"));
        in_shock = false;
        peak_g = 0;
      }

      Serial.print(millis());    Serial.print(',');
      Serial.print(imu.ax_g,3); Serial.print(',');
      Serial.print(imu.ay_g,3); Serial.print(',');
      Serial.print(imu.az_g,3); Serial.print(',');
      Serial.print(mag,3);      Serial.print(',');
      Serial.println(is_shock ? 1 : 0);

      while ((micros()-t0) < 2000UL);
    }

    mpu_write(REG_SMPLRT_DIV, 9);

  } else {
    Serial.println(F("512 mau @ 500Hz. Nhan phim de bat dau:"));
    wait_serial();

    mpu_write(REG_SMPLRT_DIV, 1);
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

      if (calib.done) { ax+=calib.ax_off; ay+=calib.ay_off; az+=calib.az_off; }

      float mag = sqrt(ax*ax+ay*ay+az*az) - 1.0f;
      Serial.println(mag,5);
      while ((micros()-t0) < 2000UL);
    }

    Serial.println(F("SAMPLE_END"));
    mpu_write(REG_SMPLRT_DIV, 9);
  }

  Serial.println(F("\nBai 5 xong. Gui '0' de ve menu."));
}

void task6_angles() {
  Serial.println(F("BAI 6: ROLL / PITCH / YAW"));
  Serial.println(F("Time_ms,Roll_Accel,Pitch_Accel,Roll_Gyro,Pitch_Gyro,Yaw_Gyro,Roll_KF,Pitch_KF,Yaw_KF"));
  Serial.println(F("Gui 'q' de dung"));
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

    Serial.print(millis());       Serial.print(',');
    Serial.print(roll_a,2);       Serial.print(',');
    Serial.print(pitch_a,2);      Serial.print(',');
    Serial.print(roll_g,2);       Serial.print(',');
    Serial.print(pitch_g,2);      Serial.print(',');
    Serial.print(yaw_g,2);        Serial.print(',');
    Serial.print(kf_roll_out,2);  Serial.print(',');
    Serial.print(kf_pitch_out,2); Serial.print(',');
    Serial.println(yaw_kf,2);

    delay(10);
  }

  Serial.println(F("\nBai 6 xong. Gui '0' de ve menu."));
}

void task7_summary() {
  Serial.println(F("BAI 7: TONG HOP"));
  Serial.println(F("Giu thiet bi yen trong 5 giay de calib gyro..."));
  delay(2000);

  bool stable  = false;
  int attempts = 0;

  while (!stable && attempts < 5) {
    double sgx=0, sgy=0, sgz=0;
    double sq_gx=0, sq_gy=0, sq_gz=0;
    for (int i = 0; i < 500; i++) {
      uint8_t buf[14];
      mpu_read_burst(REG_ACCEL_XOUT_H, buf, 14);
      float gx = (int16_t)((buf[8]<<8)|buf[9])   / GYRO_SCALE_250;
      float gy = (int16_t)((buf[10]<<8)|buf[11]) / GYRO_SCALE_250;
      float gz = (int16_t)((buf[12]<<8)|buf[13]) / GYRO_SCALE_250;
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
      Serial.print(F("Gyro offset: Gx=")); Serial.print(mgx,4);
      Serial.print(F(" Gy="));              Serial.print(mgy,4);
      Serial.print(F(" Gz="));              Serial.println(mgz,4);
    } else {
      attempts++;
      Serial.print(F("Khong on dinh (STD=")); Serial.print(std_total,3);
      Serial.print(F("), thu lai lan "));      Serial.println(attempts);
      if (attempts < 5) delay(2000);
    }
  }

  if (!stable) {
    Serial.println(F("Calib that bai sau 5 lan, dung offset cuoi."));
    calib.done = true;
  }

  read_sensor();
  float roll  = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
  float pitch = atan2f(-imu.ax_g, sqrtf(imu.ay_g*imu.ay_g+imu.az_g*imu.az_g)) * RAD_TO_DEG;
  float yaw   = 0;

  kf_roll  = {roll,  0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  kf_pitch = {pitch, 0, {{0,0},{0,0}}, 0.001f, 0.003f, 0.03f};
  last_us  = micros();

  Serial.println(F("Time_ms,Roll_KF,Pitch_KF,Yaw_Gyro,Temp_C,Accel_Mag,SHOCK"));
  Serial.println(F("Gui 'q' de dung"));

  bool in_shock = false;
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
      Serial.println(F("# SHOCK DETECTED"));
    }
    if (!shock && in_shock) {
      in_shock = false;
      Serial.print(F("# SHOCK END. ")); Serial.print(millis()-shock_t); Serial.println(F(" ms"));
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

  Serial.println(F("\nBai 7 xong. Gui '0' de ve menu."));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);
  delay(500);
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
