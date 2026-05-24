clc; clear; close all;

filename = 'bai1_data.txt';
fid = fopen(filename, 'r');
data = [];
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    line = strtrim(line);
    if isempty(line) || ~contains(line, ','), continue; end
    vals = str2double(strsplit(line, ','));
    if numel(vals) == 6 && ~any(isnan(vals))
        data(end+1, :) = vals; %#ok<AGROW>
    end
end
fclose(fid);

if isempty(data)
    error('Khong doc duoc du lieu! Kiem tra lai file: %s', filename);
end

ax = data(:,1);  ay = data(:,2);  az = data(:,3);
gx = data(:,4);  gy = data(:,5);  gz = data(:,6);
N  = length(ax);
t  = (0:N-1) * 0.01;

fprintf('=== BAI 1: DAC TINH TINH MPU6050 ===\n');
fprintf('So mau: %d | Thoi gian: %.1f s\n\n', N, t(end));

%% ---- 2. TINH MEAN & STD ----
m_ax=mean(ax); m_ay=mean(ay); m_az=mean(az);
m_gx=mean(gx); m_gy=mean(gy); m_gz=mean(gz);
s_ax=std(ax);  s_ay=std(ay);  s_az=std(az);
s_gx=std(gx);  s_gy=std(gy);  s_gz=std(gz);
mag_a = mean(sqrt(ax.^2+ay.^2+az.^2));

fprintf('[ACCELEROMETER] (don vi: g)\n');
fprintf('  Mean : Ax=%+.5f  Ay=%+.5f  Az=%+.5f\n', m_ax, m_ay, m_az);
fprintf('  STD  : Ax=%.6f  Ay=%.6f  Az=%.6f\n', s_ax, s_ay, s_az);
fprintf('[GYROSCOPE] (don vi: deg/s)\n');
fprintf('  Mean : Gx=%+.5f  Gy=%+.5f  Gz=%+.5f\n', m_gx, m_gy, m_gz);
fprintf('  STD  : Gx=%.6f  Gy=%.6f  Gz=%.6f\n', s_gx, s_gy, s_gz);
fprintf('[DANH GIA]\n');
fprintf('  |a| trung binh = %.5f g  (ly tuong: 1.000)\n', mag_a);
fprintf('  Sai lech |a|   = %.2f%%\n\n', abs(mag_a-1)*100);

if abs(m_ax)<0.05 && abs(m_ay)<0.05
    fprintf('  [OK] Zero-offset Accel X,Y trong gioi han 5%%g\n');
else
    fprintf('  [!] Zero-offset Accel lon -> can hieu chinh (Bai 2)\n');
end
if abs(m_gx)<0.5 && abs(m_gy)<0.5 && abs(m_gz)<0.5
    fprintf('  [OK] Zero-offset Gyro chap nhan duoc\n');
else
    fprintf('  [!] Gyro offset lon -> can hieu chinh (Bai 2)\n');
end
if abs(mag_a-1)<0.03
    fprintf('  [OK] |a| trong gioi han 3%%\n');
else
    fprintf('  [!] |a| lech >3%% -> can hieu chinh gain\n');
end
%% ---- 3. VE DO THI ----
fig = figure('Name','Bai 1 - Dac tinh tinh MPU6050','NumberTitle','off',...
             'Position',[50 50 1300 800]);

% --- Plot Accelerometer ---
subplot(3,2,1);
plot(t, ax, 'r', t, ay, 'g', t, az, 'b', 'LineWidth',0.8);
hold on;
yline(m_ax,'r--','LineWidth',1.2);
yline(m_ay,'g--','LineWidth',1.2);
yline(m_az,'b--','LineWidth',1.2);
yline(1,'w:','LineWidth',1);
xlabel('Thoi gian (s)'); ylabel('Gia toc (g)');
title('Gia toc tho theo thoi gian');
legend('Ax','Ay','Az','Mean Ax','Mean Ay','Mean Az','Az=1g ly tuong','Location','best');
grid on;

% --- Histogram Accel ---
subplot(3,2,2);
hold on;
histogram(ax, 30, 'FaceColor','r', 'FaceAlpha',0.5, 'EdgeColor','none');
histogram(ay, 30, 'FaceColor','g', 'FaceAlpha',0.5, 'EdgeColor','none');
histogram(az, 30, 'FaceColor','b', 'FaceAlpha',0.5, 'EdgeColor','none');
xlabel('Gia toc (g)'); ylabel('So mau');
title('Phan bo gia toc (Histogram)');
legend('Ax','Ay','Az'); grid on;

% --- Plot Gyroscope ---
subplot(3,2,3);
plot(t, gx, 'r', t, gy, 'g', t, gz, 'b', 'LineWidth',0.8);
hold on;
yline(m_gx,'r--','LineWidth',1.2);
yline(m_gy,'g--','LineWidth',1.2);
yline(m_gz,'b--','LineWidth',1.2);
yline(0,'w:','LineWidth',1);
xlabel('Thoi gian (s)'); ylabel('Van toc goc (deg/s)');
title('Gyroscope tho theo thoi gian');
legend('Gx','Gy','Gz','Mean Gx','Mean Gy','Mean Gz','0 deg/s','Location','best');
grid on;

% --- Histogram Gyro ---
subplot(3,2,4);
hold on;
histogram(gx, 30, 'FaceColor','r', 'FaceAlpha',0.5, 'EdgeColor','none');
histogram(gy, 30, 'FaceColor','g', 'FaceAlpha',0.5, 'EdgeColor','none');
histogram(gz, 30, 'FaceColor','b', 'FaceAlpha',0.5, 'EdgeColor','none');
xlabel('Van toc goc (deg/s)'); ylabel('So mau');
title('Phan bo gyroscope (Histogram)');
legend('Gx','Gy','Gz'); grid on;

% --- Gia toc tong hop |a| ---
subplot(3,2,5);
mag_vec = sqrt(ax.^2+ay.^2+az.^2);
plot(t, mag_vec, 'w', 'LineWidth',0.8);
hold on;
yline(1,   'r--', 'DisplayName','1g ly tuong', 'LineWidth',1.5);
yline(1.03,'y:',  'DisplayName','+3%',          'LineWidth',1);
yline(0.97,'y:',  'DisplayName','-3%',          'LineWidth',1);
xlabel('Thoi gian (s)'); ylabel('|a| (g)');
title(sprintf('Gia toc tong hop  |a| = %.4f g (ly tuong: 1.000)', mean(mag_vec)));
legend('|a|','1g','+3%','-3%','Location','best'); grid on;

% --- FIX: Bang tong ket bang annotation thay vi uitable ---
ax6 = subplot(3,2,6);
axis(ax6, 'off');

labels = {'Thong so', 'Mean',                     'STD (noise)';
          'Ax (g)',   sprintf('%+.5f', m_ax),      sprintf('%.6f', s_ax);
          'Ay (g)',   sprintf('%+.5f', m_ay),      sprintf('%.6f', s_ay);
          'Az (g)',   sprintf('%+.5f', m_az),      sprintf('%.6f', s_az);
          'Gx(d/s)',  sprintf('%+.5f', m_gx),      sprintf('%.6f', s_gx);
          'Gy(d/s)',  sprintf('%+.5f', m_gy),      sprintf('%.6f', s_gy);
          'Gz(d/s)',  sprintf('%+.5f', m_gz),      sprintf('%.6f', s_gz);
          '|a| (g)',  sprintf('%.5f',  mag_a),     sprintf('%.2f%%', abs(mag_a-1)*100)};

title(ax6, 'Bang tong ket dac tinh tinh', 'FontSize', 11, 'FontWeight', 'bold');

nRow = size(labels,1);
nCol = size(labels,2);
colX = [0.02, 0.38, 0.70];
rowY = linspace(0.85, 0.05, nRow); 

for r = 1:nRow
    for c = 1:nCol
        fw = 'normal';
        if r == 1, fw = 'bold'; end 
        text(colX(c), rowY(r), labels{r,c}, ...
             'Units','normalized', 'Parent', ax6, ...
             'FontSize', 9, 'FontWeight', fw, ...
             'HorizontalAlignment', 'left', ...
             'Interpreter', 'none');
    end
end

sgtitle('BAI 1 - DAC TINH TINH MPU6050', 'FontSize', 14, 'FontWeight', 'bold');

exportgraphics(fig, 'bai1_static.png', 'Resolution', 150);
fprintf('\n[OK] Da luu bai1_static.png\n');