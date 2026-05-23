%% BAI 4 - BO LOC TIN HIEU - Complementary & Kalman Filter
%  Du lieu: bai4_data.txt (xuat tu Arduino IDE)
%  Cot: Time_ms, Roll_Accel, Pitch_Accel, Roll_CF, Pitch_CF, Roll_KF, Pitch_KF

clc; clear; close all;

%% 1. DOC DU LIEU
data = readmatrix('bai4_data.txt', 'NumHeaderLines', 1);

t       = data(:,1) / 1000;   % ms -> s
roll_a  = data(:,2);           % goc Roll tu Accel (do)
pitch_a = data(:,3);           % goc Pitch tu Accel (do)
roll_cf = data(:,4);           % Roll qua Complementary Filter
pitch_cf= data(:,5);           % Pitch qua Complementary Filter
roll_kf = data(:,6);           % Roll qua Kalman Filter
pitch_kf= data(:,7);           % Pitch qua Kalman Filter

N = length(t);
fprintf('=== BAI 4: BO LOC TIN HIEU ===\n');
fprintf('So mau: %d | Thoi gian: %.2f s\n\n', N, t(end));

%% 2. THONG KE CO BAN (du lieu goc Accel - nhieu)
fprintf('--- THONG KE DU LIEU THO (Accel angle) ---\n');
fprintf('Roll_Accel : Mean = %.4f deg | STD = %.4f deg\n', mean(roll_a),  std(roll_a));
fprintf('Pitch_Accel: Mean = %.4f deg | STD = %.4f deg\n', mean(pitch_a), std(pitch_a));

fprintf('\n--- THONG KE SAU BO LOC ---\n');
fprintf('Roll_CF    : Mean = %.4f deg | STD = %.4f deg\n', mean(roll_cf),  std(roll_cf));
fprintf('Pitch_CF   : Mean = %.4f deg | STD = %.4f deg\n', mean(pitch_cf), std(pitch_cf));
fprintf('Roll_KF    : Mean = %.4f deg | STD = %.4f deg\n', mean(roll_kf),  std(roll_kf));
fprintf('Pitch_KF   : Mean = %.4f deg | STD = %.4f deg\n', mean(pitch_kf), std(pitch_kf));

%% 3. DANH GIA HIEU QUA LOC - Giam nhieu (STD reduction)
fprintf('\n--- HIEU QUA LOC (giam nhieu STD so voi Accel) ---\n');
fprintf('ROLL  : CF giam %.1f%% | KF giam %.1f%%\n', ...
    (1 - std(roll_cf)/std(roll_a))*100, ...
    (1 - std(roll_kf)/std(roll_a))*100);
fprintf('PITCH : CF giam %.1f%% | KF giam %.1f%%\n', ...
    (1 - std(pitch_cf)/std(pitch_a))*100, ...
    (1 - std(pitch_kf)/std(pitch_a))*100);

%% 4. PHAN TICH DRIFT GYRO
%  Drift the hien qua su chenh lech tich luy cua KF so voi Accel
drift_roll  = roll_kf(end)  - roll_kf(1);
drift_pitch = pitch_kf(end) - pitch_kf(1);
T_total = t(end) - t(1);
fprintf('\n--- DRIFT GYRO (uoc tinh) ---\n');
fprintf('Roll  drift tong: %.4f deg trong %.2f s = %.4f deg/s\n', drift_roll,  T_total, drift_roll/T_total);
fprintf('Pitch drift tong: %.4f deg trong %.2f s = %.4f deg/s\n', drift_pitch, T_total, drift_pitch/T_total);

%% 5. SO SANH SAI LECH GIUA CF VA KF
diff_roll  = roll_cf  - roll_kf;
diff_pitch = pitch_cf - pitch_kf;
fprintf('\n--- SAI LECH GIUA CF VA KF ---\n');
fprintf('Roll  : Max diff = %.4f deg | RMS diff = %.4f deg\n', max(abs(diff_roll)),  rms(diff_roll));
fprintf('Pitch : Max diff = %.4f deg | RMS diff = %.4f deg\n', max(abs(diff_pitch)), rms(diff_pitch));

%% 6. VE DO THI

% --- Figure 1: Roll - So sanh 3 phuong phap ---
figure('Name','BAI 4 - ROLL ANGLE COMPARISON','NumberTitle','off');
subplot(2,1,1);
plot(t, roll_a,  'Color',[0.7 0.7 0.7], 'LineWidth',0.8, 'DisplayName','Raw Accel');
hold on;
plot(t, roll_cf, 'b-', 'LineWidth',1.2, 'DisplayName',sprintf('Complementary (alpha=0.98)'));
plot(t, roll_kf, 'r-', 'LineWidth',1.5, 'DisplayName','Kalman Filter');
hold off;
xlabel('Time (s)'); ylabel('Roll (deg)');
title('Roll Angle - Raw Accel vs Complementary vs Kalman');
legend('Location','best'); grid on;

subplot(2,1,2);
plot(t, pitch_a,  'Color',[0.7 0.7 0.7], 'LineWidth',0.8, 'DisplayName','Raw Accel');
hold on;
plot(t, pitch_cf, 'b-', 'LineWidth',1.2, 'DisplayName','Complementary');
plot(t, pitch_kf, 'r-', 'LineWidth',1.5, 'DisplayName','Kalman Filter');
hold off;
xlabel('Time (s)'); ylabel('Pitch (deg)');
title('Pitch Angle - Raw Accel vs Complementary vs Kalman');
legend('Location','best'); grid on;

% --- Figure 2: Phan phoi nhieu (Histogram) ---
figure('Name','BAI 4 - NOISE DISTRIBUTION','NumberTitle','off');
subplot(2,3,1); histogram(roll_a,  40); title('Roll Raw Accel'); xlabel('deg'); grid on;
subplot(2,3,2); histogram(roll_cf, 40); title('Roll CF');        xlabel('deg'); grid on;
subplot(2,3,3); histogram(roll_kf, 40); title('Roll KF');        xlabel('deg'); grid on;
subplot(2,3,4); histogram(pitch_a,  40); title('Pitch Raw Accel'); xlabel('deg'); grid on;
subplot(2,3,5); histogram(pitch_cf, 40); title('Pitch CF');        xlabel('deg'); grid on;
subplot(2,3,6); histogram(pitch_kf, 40); title('Pitch KF');        xlabel('deg'); grid on;
sgtitle('Noise Distribution Comparison');

% --- Figure 3: Pho tan so (FFT) cua Roll ---
figure('Name','BAI 4 - FFT SPECTRUM','NumberTitle','off');
fs = 1 / mean(diff(t));   % tan so lay mau (Hz)
L  = N;
f  = (0:L/2) * fs / L;

Y_raw = abs(fft(roll_a  - mean(roll_a)));   Y_raw  = Y_raw(1:L/2+1);
Y_cf  = abs(fft(roll_cf - mean(roll_cf)));  Y_cf   = Y_cf(1:L/2+1);
Y_kf  = abs(fft(roll_kf - mean(roll_kf)));  Y_kf   = Y_kf(1:L/2+1);

semilogy(f, Y_raw, 'Color',[0.7 0.7 0.7], 'LineWidth',0.8, 'DisplayName','Raw Accel');
hold on;
semilogy(f, Y_cf,  'b-', 'LineWidth',1.2, 'DisplayName','Complementary');
semilogy(f, Y_kf,  'r-', 'LineWidth',1.5, 'DisplayName','Kalman');
hold off;
xlabel('Frequency (Hz)'); ylabel('Magnitude');
title('FFT Spectrum - Roll Angle (so sanh loc nhieu tan so cao)');
legend('Location','best'); grid on;
xlim([0, fs/2]);

% --- Figure 4: Sai lech CF vs KF theo thoi gian ---
figure('Name','BAI 4 - CF vs KF DIFFERENCE','NumberTitle','off');
subplot(2,1,1);
plot(t, diff_roll,  'm-', 'LineWidth',1);
xlabel('Time (s)'); ylabel('Deg');
title('Roll: CF - KF difference over time'); grid on;
yline(0,'k--');

subplot(2,1,2);
plot(t, diff_pitch, 'c-', 'LineWidth',1);
xlabel('Time (s)'); ylabel('Deg');
title('Pitch: CF - KF difference over time'); grid on;
yline(0,'k--');

%% 7. BANG TONG HOP KET QUA
fprintf('\n=== BANG TONG HOP KET QUA BAI 4 ===\n');
fprintf('%-15s | %-10s | %-10s | %-10s\n', 'Chi tieu','Du lieu tho','Comp.Filter','Kalman Filter');
fprintf('%s\n', repmat('-',1,55));
fprintf('%-15s | %-10.4f | %-10.4f | %-10.4f\n', 'Roll STD (deg)', std(roll_a),  std(roll_cf),  std(roll_kf));
fprintf('%-15s | %-10.4f | %-10.4f | %-10.4f\n', 'Pitch STD (deg)',std(pitch_a), std(pitch_cf), std(pitch_kf));
fprintf('%-15s | %-10s | %-10s | %-10s\n', 'Chong drift','Khong','Tot','Rat tot');
fprintf('%-15s | %-10s | %-10s | %-10s\n', 'Do phuc tap','Toi thieu','Thap','Trung binh');
fprintf('\n');

fprintf('Script hoan tat. Kiem tra cac Figure de xem do thi.\n');