clear; clc; close all;
fid = fopen('bai_6.txt', 'r');
raw = textscan(fid, '%f%f%f%f%f%f%f%f%f', ...
    'Delimiter', ',', 'CommentStyle', '#', ...
    'HeaderLines', 3);
fclose(fid);

t_ms    = raw{1};
Ra      = raw{2};
Pa      = raw{3};
Rg      = raw{4};
Pg      = raw{5};
Yg      = raw{6};
Rkf     = raw{7};
Pkf     = raw{8};
Ykf     = raw{9};
t = t_ms / 1000; 
fprintf('Du lieu: %d mau | %.1f s | Fs = %.0f Hz\n', ...
    length(t), t(end)-t(1), 1/mean(diff(t)));

%% ===== 2. VE DO THI TONG QUAT =====
figure('Name','Bai 6 - Roll / Pitch / Yaw', 'Position',[100 100 1200 800]);

subplot(3,1,1);
plot(t, Ra, 'Color',[0.7 0.7 0.7], 'DisplayName','Accel');
hold on;
plot(t, Rg, 'b--', 'DisplayName','Gyro');
plot(t, Rkf, 'r', 'LineWidth',1.5, 'DisplayName','Kalman');
yline(45,'y:','45°','LabelVerticalAlignment','bottom');
yline(0,'y:');
xlabel('Thoi gian (s)'); ylabel('Goc (do)');
title('ROLL'); legend('Location','best'); grid on;

subplot(3,1,2);
plot(t, Pa, 'Color',[0.7 0.7 0.7], 'DisplayName','Accel');
hold on;
plot(t, Pg, 'b--', 'DisplayName','Gyro');
plot(t, Pkf, 'r', 'LineWidth',1.5, 'DisplayName','Kalman');
yline(30,'y:','30°','LabelVerticalAlignment','bottom');
yline(0,'y:');
xlabel('Thoi gian (s)'); ylabel('Goc (do)');
title('PITCH'); legend('Location','best'); grid on;

subplot(3,1,3);
plot(t, Yg, 'b--', 'DisplayName','Gyro');
hold on;
plot(t, Ykf, 'r', 'LineWidth',1.5, 'DisplayName','Kalman');
xlabel('Thoi gian (s)'); ylabel('Goc (do)');
title('YAW (drift ro vi khong co Magnetometer)');
legend('Location','best'); grid on;

saveas(gcf, 'bai6_overview.png');

%% ===== 3. DANH GIA DO ON DINH KHI GIU ROLL 45 DO (5s - 15s) =====
mask_hold = (t >= 5) & (t <= 15);

mean_Rkf_hold = mean(Rkf(mask_hold));
std_Rkf_hold  = std(Rkf(mask_hold));
max_dev_hold  = max(abs(Rkf(mask_hold) - 45));

fprintf('\n--- ON DINH KHI GIU ROLL 45 do (5-15s) ---\n');
fprintf('  Gia tri trung binh KF  : %.3f do\n', mean_Rkf_hold);
fprintf('  Do lech khoi 45 do     : %.3f do\n', mean_Rkf_hold - 45);
fprintf('  STD (nhieu)            : %.4f do\n', std_Rkf_hold);
fprintf('  Sai so dinh (max dev)  : %.3f do\n', max_dev_hold);

if max_dev_hold < 0.5
    fprintf('  => RAT TOT (< 0.5 do)\n');
elseif max_dev_hold < 1.0
    fprintf('  => TOT (< 1 do)\n');
else
    fprintf('  => CAN KIEM TRA (> 1 do)\n');
end

%% ===== 4. DANH GIA HOI TU VE 0 SAU KHI QUAY (15s - 17s) =====
mask_ret = (t >= 15) & (t <= 20);
t_ret    = t(mask_ret);
R_ret    = Rkf(mask_ret);

idx_conv = find(abs(R_ret) < 1.0, 1, 'first');
if ~isempty(idx_conv)
    t_conv = t_ret(idx_conv) - 15;
    fprintf('\n--- HOI TU VE 0 SAU KHI THA ROLL (bat dau t=15s) ---\n');
    fprintf('  Thoi gian hoi tu vao ±1 do : %.2f s\n', t_conv);
    if t_conv <= 2
        fprintf('  => XUAT SAC (< 2s)\n');
    elseif t_conv <= 3
        fprintf('  => TOT (< 3s)\n');
    else
        fprintf('  => CHAM (> 3s)\n');
    end
else
    fprintf('\n  [!] Khong hoi tu ve 0 trong doan 15-20s\n');
end

%% ===== 5. SO SANH VOI GOC CHUAN (0, 30, 45 do) =====
fprintf('\n--- SO SANH VOI GOC CHUAN ---\n');
fprintf('%-12s %-12s %-12s %-12s %-12s\n', ...
    'Goc chuan','t_start','Mean_KF','Error(do)','Danh gia');

ref_angles = [0, 45, 0, 30, 0];
ref_tstart = [1, 5, 17, 22, 27];
ref_tend   = [3, 15, 20, 27, 29];

for i = 1:length(ref_angles)
    mask_i = (t >= ref_tstart(i)) & (t <= ref_tend(i));
    if ref_angles(i) == 0
        val = mean(Rkf(mask_i));
        if i >= 4
            val = mean(Pkf(mask_i));
        end
    elseif ref_angles(i) == 45
        val = mean(Rkf(mask_i));
    elseif ref_angles(i) == 30
        val = mean(Pkf(mask_i));
    end
    err = abs(val - ref_angles(i));
    if err < 0.5
        grade = 'Rat tot';
    elseif err < 1.0
        grade = 'Tot';
    else
        grade = 'Can check';
    end
    fprintf('%-12d %-12.1f %-12.3f %-12.3f %-12s\n', ...
        ref_angles(i), ref_tstart(i), val, err, grade);
end

%% ===== 6. DANH GIA DO TRE (LATENCY) =====
t_roll = t((t>=3) & (t<=6));
R_roll_kf = Rkf((t>=3) & (t<=6));
R_roll_ac = Ra((t>=3) & (t<=6));
idx_ac  = find(R_roll_ac > 5, 1, 'first');
idx_kf  = find(R_roll_kf > 5, 1, 'first');
if ~isempty(idx_ac) && ~isempty(idx_kf)
    latency_ms = (t_roll(idx_kf) - t_roll(idx_ac)) * 1000;
    fprintf('\n--- DO TRE (LATENCY) ---\n');
    fprintf('  Accel vuot 5 do tai t = %.3f s\n', t_roll(idx_ac));
    fprintf('  KF vuot 5 do tai    t = %.3f s\n', t_roll(idx_kf));
    fprintf('  Do tre KF so Accel    : %.1f ms\n', latency_ms);
    if abs(latency_ms) < 10
        fprintf('  => RAT THAP (< 10ms)\n');
    elseif abs(latency_ms) < 50
        fprintf('  => CHAP NHAN DUOC (< 50ms)\n');
    else
        fprintf('  => CAO, can chinh lai Q/R\n');
    end
end

%% ===== 7. DO DRIFT YAW (29-35s) =====
mask_yaw = (t >= 29) & (t <= 35);
yaw_start = Yg(find(mask_yaw, 1, 'first'));
yaw_end   = Yg(find(mask_yaw, 1, 'last'));
drift_rate = (yaw_end - yaw_start) / 6;
fprintf('\n--- DRIFT YAW (29-35s, khong co Magnetometer) ---\n');
fprintf('  Toc do drift         : %.3f do/s\n', drift_rate);
fprintf('  Drift sau 6s         : %.2f do\n', yaw_end - yaw_start);
fprintf('  Note: can Magnetometer de khoa Yaw\n');

%% ===== 8. VE DO THI PHAN TICH RO HAN =====
figure('Name','Phan tich chi tiet', 'Position',[100 100 1200 500]);

subplot(1,2,1);
mask_p1 = (t >= 4) & (t <= 17);
plot(t(mask_p1), Ra(mask_p1), 'Color',[0.7 0.7 0.7], 'DisplayName','Accel');
hold on;
plot(t(mask_p1), Rkf(mask_p1), 'r', 'LineWidth',1.5, 'DisplayName','Kalman KF');
yline(45,'y--','45° chuan'); yline(0,'y--');
xlabel('t (s)'); ylabel('Roll (do)');
title('ROLL: tang - giu - ve'); legend; grid on;

subplot(1,2,2);
mask_p2 = (t >= 19) & (t <= 35);
yyaxis left;
plot(t(mask_p2), Pkf(mask_p2), 'r', 'LineWidth',1.5, 'DisplayName','Pitch KF');
ylabel('Pitch (do)');
yyaxis right;
plot(t(mask_p2), Yg(mask_p2), 'b--', 'DisplayName','Yaw Gyro');
ylabel('Yaw (do)');
xlabel('t (s)');
title('PITCH + YAW (drift)'); grid on; legend;

saveas(gcf, 'bai6_detail.png');

fprintf('\n=== HOAN TAT PHAN TICH BAI 6 ===\n');
fprintf('Da luu hinh: bai6_overview.png, bai6_detail.png\n');