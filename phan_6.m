clear; close all; clc;

%% ---------------------------------------------------------------
%  CAU HINH - THAY DOI O DAY
% ---------------------------------------------------------------
FILE_PATH = 'bai_6.txt';   % Duong dan toi file du lieu
FS        = 100;                 % Tan so lay mau (Hz) - 100Hz theo config

% Neu muon phan tich mot doan cu the (giay), dat: TRIM_SEC = [t_start t_end]
% Vi du: TRIM_SEC = [2 60]; de bo 2 giay dau (khoi dong) va lay 60 giay
% De lay toan bo: TRIM_SEC = [];
TRIM_SEC = [];

%% ---------------------------------------------------------------
%  DOC DU LIEU
% ---------------------------------------------------------------
fprintf('Doc file: %s\n', FILE_PATH);

% Doc file, bo dong bat dau bang '#' (comment) va dong header
fid = fopen(FILE_PATH, 'r');
if fid < 0
    error('Khong mo duoc file: %s\nKiem tra lai duong dan.', FILE_PATH);
end

raw_lines = {};
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if ischar(line) && ~isempty(line) && line(1) ~= '#'
        raw_lines{end+1} = line; %#ok<AGROW>
    end
end
fclose(fid);

% Bo dong header (dong dau chua chu)
data_lines = {};
for i = 1:numel(raw_lines)
    % Thu parse dong dau tien cua tung dong
    parts = strsplit(raw_lines{i}, ',');
    if numel(parts) >= 13
        val = str2double(parts{1});
        if ~isnan(val)
            data_lines{end+1} = raw_lines{i}; %#ok<AGROW>
        end
    end
end

if isempty(data_lines)
    error('Khong tim thay du lieu hop le trong file. Kiem tra dinh dang CSV.');
end

% Chuyen sang ma tran so
N = numel(data_lines);
D = zeros(N, 13);
for i = 1:N
    parts = strsplit(data_lines{i}, ',');
    for j = 1:min(13, numel(parts))
        D(i,j) = str2double(parts{j});
    end
end

% Loai NaN
D = D(~any(isnan(D), 2), :);
N = size(D, 1);
fprintf('So mau hop le: %d\n', N);

% Lay cac cot
t_ms    = D(:,1);
t_s     = (t_ms - t_ms(1)) / 1000;  % Chuyen ve giay, bat dau tu 0

Roll_A  = D(:,2);   % Accelerometer
Pitch_A = D(:,3);
Roll_G  = D(:,4);   % Gyro integration
Pitch_G = D(:,5);
Yaw_G   = D(:,6);
Roll_CF = D(:,7);   % Complementary Filter
Pitch_CF= D(:,8);
Roll_KF = D(:,9);   % Kalman Filter
Pitch_KF= D(:,10);
Ax_lin  = D(:,11);  % Linear acceleration (g)
Ay_lin  = D(:,12);
Az_lin  = D(:,13);

% Trim theo thoi gian neu can
if ~isempty(TRIM_SEC)
    idx = t_s >= TRIM_SEC(1) & t_s <= TRIM_SEC(2);
    if sum(idx) < 10
        warning('TRIM_SEC co qua it mau, bo qua trim.');
    else
        t_s     = t_s(idx) - t_s(find(idx,1));
        Roll_A  = Roll_A(idx);  Pitch_A  = Pitch_A(idx);
        Roll_G  = Roll_G(idx);  Pitch_G  = Pitch_G(idx);
        Yaw_G   = Yaw_G(idx);
        Roll_CF = Roll_CF(idx); Pitch_CF = Pitch_CF(idx);
        Roll_KF = Roll_KF(idx); Pitch_KF = Pitch_KF(idx);
        Ax_lin  = Ax_lin(idx);  Ay_lin   = Ay_lin(idx);
        Az_lin  = Az_lin(idx);
        N = sum(idx);
        fprintf('Sau trim: %d mau (%.1f - %.1f giay)\n', N, TRIM_SEC(1), TRIM_SEC(2));
    end
end

T_total = t_s(end);
fprintf('Tong thoi gian: %.1f giay\n', T_total);

%% ---------------------------------------------------------------
%  TINH THONG KE
% ---------------------------------------------------------------
fprintf('\n========== THONG KE ==========\n');

% Drift rate cua Gyro (do/giay)
drift_roll  = (Roll_G(end)  - Roll_KF(end))  / T_total;
drift_pitch = (Pitch_G(end) - Pitch_KF(end)) / T_total;
drift_yaw   = Yaw_G(end) / T_total;  % Yaw luon drift, khong co reference

fprintf('DRIFT GYRO:\n');
fprintf('  Roll  Gyro drift rate : %.4f deg/s (%.2f deg/phut)\n', drift_roll,  drift_roll*60);
fprintf('  Pitch Gyro drift rate : %.4f deg/s (%.2f deg/phut)\n', drift_pitch, drift_pitch*60);
fprintf('  Yaw   total drift     : %.2f deg trong %.1fs\n',        Yaw_G(end), T_total);

% RMSE so sanh CF vs KF (dung KF lam reference)
rmse_roll_cf  = sqrt(mean((Roll_CF  - Roll_KF ).^2));
rmse_pitch_cf = sqrt(mean((Pitch_CF - Pitch_KF).^2));
rmse_roll_a   = sqrt(mean((Roll_A   - Roll_KF ).^2));
rmse_pitch_a  = sqrt(mean((Pitch_A  - Pitch_KF).^2));
rmse_roll_g   = sqrt(mean((Roll_G   - Roll_KF ).^2));
rmse_pitch_g  = sqrt(mean((Pitch_G  - Pitch_KF).^2));

fprintf('\nRMSE SO VIET KF (tham chieu):\n');
fprintf('  Roll  - Accel : %.3f deg | Gyro : %.3f deg | CF : %.3f deg\n', ...
        rmse_roll_a, rmse_roll_g, rmse_roll_cf);
fprintf('  Pitch - Accel : %.3f deg | Gyro : %.3f deg | CF : %.3f deg\n', ...
        rmse_pitch_a, rmse_pitch_g, rmse_pitch_cf);

% Bien do nhieu Accelerometer vs Kalman
noise_roll  = std(Roll_A  - Roll_KF);
noise_pitch = std(Pitch_A - Pitch_KF);
fprintf('\nNHIEU ACCEL (STD so KF):\n');
fprintf('  Roll STD  : %.3f deg\n', noise_roll);
fprintf('  Pitch STD : %.3f deg\n', noise_pitch);

% Gia toc tuyen tinh
rms_ax = rms(Ax_lin);
rms_ay = rms(Ay_lin);
rms_az = rms(Az_lin);
fprintf('\nGIA TOC TUYEN TINH (RMS, should~0 khi dung yen):\n');
fprintf('  Ax_lin RMS : %.4f g\n', rms_ax);
fprintf('  Ay_lin RMS : %.4f g\n', rms_ay);
fprintf('  Az_lin RMS : %.4f g\n', rms_az);
fprintf('================================\n');

%% ---------------------------------------------------------------
%  HINH 1: GOCROLL - SO SANH 4 PHUONG PHAP
% ---------------------------------------------------------------
figure('Name','H1: Goc Roll','NumberTitle','off','Position',[50 550 900 420]);

subplot(2,1,1);
plot(t_s, Roll_A,  'Color',[0.5 0.5 0.5], 'LineWidth',0.8); hold on;
plot(t_s, Roll_G,  'r--', 'LineWidth',1);
plot(t_s, Roll_CF, 'b',   'LineWidth',1.5);
plot(t_s, Roll_KF, 'g',   'LineWidth',2);
hold off;
legend('Accel (nhieu cao tan)','Gyro (drift)','Comp. Filter','Kalman Filter', ...
       'Location','best','FontSize',8);
xlabel('Thoi gian (s)'); ylabel('Roll (deg)');
title('ROLL: So sanh 4 phuong phap'); grid on;

subplot(2,1,2);
plot(t_s, Roll_A - Roll_KF,  'Color',[0.6 0.6 0.6]); hold on;
plot(t_s, Roll_G - Roll_KF,  'r--');
plot(t_s, Roll_CF - Roll_KF, 'b');
hold off; yline(0,'g:');
legend('Accel - KF','Gyro - KF','CF - KF','Location','best','FontSize',8);
xlabel('Thoi gian (s)'); ylabel('Sai so (deg)');
title('Sai so Roll so voi Kalman Filter'); grid on;

%% ---------------------------------------------------------------
%  HINH 2: GOC PITCH - SO SANH 4 PHUONG PHAP
% ---------------------------------------------------------------
figure('Name','H2: Goc Pitch','NumberTitle','off','Position',[960 550 900 420]);

subplot(2,1,1);
plot(t_s, Pitch_A,  'Color',[0.5 0.5 0.5], 'LineWidth',0.8); hold on;
plot(t_s, Pitch_G,  'r--', 'LineWidth',1);
plot(t_s, Pitch_CF, 'b',   'LineWidth',1.5);
plot(t_s, Pitch_KF, 'g',   'LineWidth',2);
hold off;
legend('Accel (nhieu)','Gyro (drift)','Comp. Filter','Kalman Filter', ...
       'Location','best','FontSize',8);
xlabel('Thoi gian (s)'); ylabel('Pitch (deg)');
title('PITCH: So sanh 4 phuong phap'); grid on;

subplot(2,1,2);
plot(t_s, Pitch_A - Pitch_KF,  'Color',[0.6 0.6 0.6]); hold on;
plot(t_s, Pitch_G - Pitch_KF,  'r--');
plot(t_s, Pitch_CF - Pitch_KF, 'b');
hold off; yline(0,'g:');
legend('Accel - KF','Gyro - KF','CF - KF','Location','best','FontSize',8);
xlabel('Thoi gian (s)'); ylabel('Sai so (deg)');
title('Sai so Pitch so voi Kalman Filter'); grid on;

%% ---------------------------------------------------------------
%  HINH 3: YAW - DRIFT GYRO
% ---------------------------------------------------------------
figure('Name','H3: Yaw & Drift','NumberTitle','off','Position',[50 80 900 380]);

subplot(2,1,1);
plot(t_s, Yaw_G, 'm', 'LineWidth',1.5);
xlabel('Thoi gian (s)'); ylabel('Yaw (deg)');
title(sprintf('YAW (tich phan Gyro) - Drift %.4f deg/s', drift_yaw));
grid on;

% Ve drift tuyen tinh bieu dien
hold on;
p_drift = polyfit(t_s, Yaw_G, 1);
plot(t_s, polyval(p_drift, t_s), 'g--', 'LineWidth',1.2);
legend('Yaw Gyro', sprintf('Drift fit: %.4f t + %.2f', p_drift(1), p_drift(2)), ...
       'Location','best','FontSize',8);
hold off;

subplot(2,1,2);
% Yaw sau khi tru drift
Yaw_detrended = Yaw_G - polyval(p_drift, t_s);
plot(t_s, Yaw_detrended, 'Color',[0.2 0.6 0.2], 'LineWidth',1.2);
xlabel('Thoi gian (s)'); ylabel('Yaw tru drift (deg)');
title('Yaw sau khi loai bo drift tuyen tinh'); grid on;
yline(0,'g:');

%% ---------------------------------------------------------------
%  HINH 4: GIA TOC TUYEN TINH (6.4)
% ---------------------------------------------------------------
figure('Name','H4: Gia toc tuyen tinh','NumberTitle','off','Position',[960 80 900 520]);

subplot(3,1,1);
plot(t_s, Ax_lin, 'r', 'LineWidth',1);
xlabel('Thoi gian (s)'); ylabel('Ax\_lin (g)');
title(sprintf('GIA TOC TUYEN TINH X (da loai bo trong truong) | RMS=%.4fg', rms_ax));
grid on; yline(0,'g:');

subplot(3,1,2);
plot(t_s, Ay_lin, 'g', 'LineWidth',1);
xlabel('Thoi gian (s)'); ylabel('Ay\_lin (g)');
title(sprintf('GIA TOC TUYEN TINH Y | RMS=%.4fg', rms_ay));
grid on; yline(0,'g:');

subplot(3,1,3);
plot(t_s, Az_lin, 'b', 'LineWidth',1);
xlabel('Thoi gian (s)'); ylabel('Az\_lin (g)');
title(sprintf('GIA TOC TUYEN TINH Z | RMS=%.4fg', rms_az));
grid on; yline(0,'g:');

%% ---------------------------------------------------------------
%  HINH 5: PHAN TICH TAN SO - ACCEL VS KF (Roll)
% ---------------------------------------------------------------
figure('Name','H5: Pho tan so Roll','NumberTitle','off','Position',[50 80 850 400]);

% Tinh PSD bang Welch
nfft   = min(512, floor(N/4));
window = hanning(nfft);
[psd_a, f_a] = pwelch(Roll_A  - mean(Roll_A),  window, nfft/2, nfft, FS);
[psd_g, ~  ] = pwelch(Roll_G  - mean(Roll_G),  window, nfft/2, nfft, FS);
[psd_cf,~  ] = pwelch(Roll_CF - mean(Roll_CF), window, nfft/2, nfft, FS);
[psd_kf,~  ] = pwelch(Roll_KF - mean(Roll_KF), window, nfft/2, nfft, FS);

semilogy(f_a, psd_a,  'Color',[0.6 0.6 0.6]); hold on;
semilogy(f_a, psd_g,  'r--');
semilogy(f_a, psd_cf, 'b',  'LineWidth',1.5);
semilogy(f_a, psd_kf, 'g',  'LineWidth',2);
hold off;
legend('Accel','Gyro','CF','Kalman','Location','best','FontSize',8);
xlabel('Tan so (Hz)'); ylabel('PSD (deg^2/Hz)');
title('PHO CONG SUAT MAT DO (Welch) - Roll');
grid on; xlim([0 FS/2]);

%% ---------------------------------------------------------------
%  HINH 6: BANG SO SANH TONG HOP
% ---------------------------------------------------------------
figure('Name','H6: Tong hop','NumberTitle','off','Position',[50 200 700 500]);
axis off;

methods_str = {'Accelerometer','Gyro Integ.','Comp. Filter (CF)','Kalman Filter (KF)'};
rmse_roll_vals  = [rmse_roll_a,  rmse_roll_g,  rmse_roll_cf,  0];
rmse_pitch_vals = [rmse_pitch_a, rmse_pitch_g, rmse_pitch_cf, 0];
drift_roll_vals = [NaN, drift_roll, NaN, NaN];
drift_pitch_vals= [NaN, drift_pitch, NaN, NaN];

% In bang van ban
text_data = {};
text_data{1} = sprintf('%-22s | %10s | %10s | %12s', 'Phuong phap', 'RMSE Roll', 'RMSE Pitch', 'Drift Roll');
text_data{2} = repmat('-',1,60);
for i = 1:4
    if i == 4
        text_data{end+1} = sprintf('%-22s | %8.3f   | %8.3f   | %10s  ', ...
            methods_str{i}, rmse_roll_vals(i), rmse_pitch_vals(i), 'Reference');
    elseif isnan(drift_roll_vals(i))
        text_data{end+1} = sprintf('%-22s | %8.3f   | %8.3f   | %10s  ', ...
            methods_str{i}, rmse_roll_vals(i), rmse_pitch_vals(i), 'N/A');
    else
        text_data{end+1} = sprintf('%-22s | %8.3f   | %8.3f   | %+.4f/s ', ...
            methods_str{i}, rmse_roll_vals(i), rmse_pitch_vals(i), drift_roll_vals(i));
    end
end
text_data{end+1} = repmat('-',1,60);
text_data{end+1} = '';
text_data{end+1} = sprintf('Tong thoi gian     : %.1f giay', T_total);
text_data{end+1} = sprintf('So mau             : %d', N);
text_data{end+1} = sprintf('Tan so lay mau     : ~%d Hz', FS);
text_data{end+1} = '';
text_data{end+1} = sprintf('Nhieu Accel - Roll  STD: %.3f deg', noise_roll);
text_data{end+1} = sprintf('Nhieu Accel - Pitch STD: %.3f deg', noise_pitch);
text_data{end+1} = '';
text_data{end+1} = 'Tieu chi danh gia (Phan 6.3):';
text_data{end+1} = sprintf('  KF Roll  RMSE < 1 deg : %s', ternary(rmse_roll_cf  < 1, 'DAT', 'CHUA DAT'));
text_data{end+1} = sprintf('  KF Pitch RMSE < 1 deg : %s', ternary(rmse_pitch_cf < 1, 'DAT', 'CHUA DAT'));
text_data{end+1} = sprintf('  Drift < 0.05 deg/s    : %s', ternary(abs(drift_roll) < 0.05, 'DAT', 'CHUA DAT'));

y_pos = 0.95;
for i = 1:numel(text_data)
    text(0.02, y_pos, text_data{i}, 'FontName','Courier New', ...
         'FontSize',9, 'Units','normalized', 'VerticalAlignment','top');
    y_pos = y_pos - 0.055;
end
title('BAO CAO TONG HOP - BAI 6', 'FontSize',12, 'FontWeight','bold');

%% ---------------------------------------------------------------
%  HINH 7: QUYDAO 3D (Roll, Pitch, Yaw_KF) NEU CO CHUYEN DONG
% ---------------------------------------------------------------
figure('Name','H7: Quy dao 3D','NumberTitle','off','Position',[960 200 700 450]);

% Ve quy dao theo thoi gian duoi dang 3D color-coded
cmap = jet(N);
for i = 1:N-1
    plot3(Roll_KF(i:i+1), Pitch_KF(i:i+1), t_s(i:i+1), ...
          'Color', cmap(i,:), 'LineWidth', 1.2);
    hold on;
end
hold off;
xlabel('Roll KF (deg)'); ylabel('Pitch KF (deg)'); zlabel('Thoi gian (s)');
title('QUY DAO TU THE (Roll-Pitch-Yaw theo thoi gian)');
grid on; view(45, 30);
colormap(jet); colorbar; ylabel(colorbar,'Thoi gian (s)');

%% ---------------------------------------------------------------
%  HINH 8: GIA TOC TUYEN TINH - PHAN TICH
% ---------------------------------------------------------------
figure('Name','H8: Phan tich Gia toc tuyen tinh','NumberTitle','off','Position',[50 80 900 400]);

Alin_mag = sqrt(Ax_lin.^2 + Ay_lin.^2 + Az_lin.^2);

subplot(2,1,1);
plot(t_s, Ax_lin, 'r', 'LineWidth',0.8); hold on;
plot(t_s, Ay_lin, 'g', 'LineWidth',0.8);
plot(t_s, Az_lin, 'b', 'LineWidth',0.8);
plot(t_s, Alin_mag, 'g', 'LineWidth',1.5);
hold off;
legend('Ax\_lin','Ay\_lin','Az\_lin','|A\_lin|','Location','best','FontSize',8);
xlabel('Thoi gian (s)'); ylabel('Gia toc (g)');
title('GIA TOC TUYEN TINH (da loai tru vector trong truong bang goc KF)');
grid on; yline(0,'g:');

subplot(2,1,2);
% Histogram phan phoi gia toc tuyen tinh
histogram(Alin_mag, 50, 'Normalization','probability', 'FaceColor',[0.3 0.5 0.8]);
xlabel('|A\_linear| (g)'); ylabel('Xac suat');
title('Phan phoi bieu do bien do gia toc tuyen tinh');
xline(mean(Alin_mag), 'r--', sprintf('Mean=%.4fg', mean(Alin_mag)), ...
      'LabelVerticalAlignment','bottom');
grid on;

fprintf('\n=== XU LY HOAN TAT ===\n');
fprintf('Da tao 8 hinh ve. Ket qua tong hop o Figure 6.\n');

%% ---------------------------------------------------------------
%  HAM PHU TRO
% ---------------------------------------------------------------
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end