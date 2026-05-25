% ================================================================
%  BAI 6 - UOC LUONG GOC ROLL / PITCH / YAW
%  Cam bien: MPU6050 (Accel + Gyro)
%  Kalman Filter da chay tren Arduino, MATLAB phan tich ket qua
% ================================================================
clear; clc; close all;

% ================================================================
%  MAU SAC CHO GIAO DIEN TOI (Dark Theme)
% ================================================================
CLR_BG      = [0.10  0.10  0.10];   % nen figure
CLR_AX      = [0.15  0.15  0.15];   % nen axes
CLR_TXT     = [0.95  0.95  0.95];   % chu trang
CLR_GRID    = [0.30  0.30  0.30];   % luoi
CLR_ACCEL   = [0.55  0.55  0.55];   % xam
CLR_GYRO    = [0.40  0.70  1.00];   % xanh duong nhat
CLR_KF      = [1.00  0.35  0.35];   % do
CLR_KF_P    = [0.35  0.85  0.55];   % xanh la (Pitch KF)
CLR_REF     = [1.00  0.85  0.20];   % vang (duong chuan)
CLR_HOLD_R  = [0.25  0.22  0.10];   % vung giu Roll
CLR_HOLD_P  = [0.10  0.20  0.28];   % vung giu Pitch
CLR_WARN    = [0.90  0.50  0.10];   % cam (canh bao)

%% ===== 1. DOC DU LIEU =====
fid = fopen('bai_6.txt', 'r');
raw = textscan(fid, '%f%f%f%f%f%f%f%f%f', ...
    'Delimiter', ',', 'CommentStyle', '#', 'HeaderLines', 3);
fclose(fid);

t_ms = raw{1};
Ra   = raw{2};   % Roll  - Accelerometer
Pa   = raw{3};   % Pitch - Accelerometer
Rg   = raw{4};   % Roll  - Gyro
Pg   = raw{5};   % Pitch - Gyro
Yg   = raw{6};   % Yaw   - Gyro (tich phan)
Rkf  = raw{7};   % Roll  - Kalman Filter
Pkf  = raw{8};   % Pitch - Kalman Filter

t       = t_ms / 1000;
dt_mean = mean(diff(t));
Fs      = 1 / dt_mean;

fprintf('========================================\n');
fprintf('  DU LIEU BAI 6\n');
fprintf('========================================\n');
fprintf('  So mau          : %d\n',    length(t));
fprintf('  Thoi gian tong  : %.2f s\n', t(end)-t(1));
fprintf('  Tan so lay mau  : %.1f Hz\n', Fs);
fprintf('  dt trung binh   : %.2f ms\n', dt_mean*1000);

%% ===== 2. MOC THOI GIAN =====
T_STATIC_S    = 0.5;   T_STATIC_E    = 2.5;
T_ROLL_START  = 3.0;
T_ROLL_HOLD_S = 5.0;   T_ROLL_HOLD_E = 15.0;
T_ROLL_END    = 17.0;
T_REST1_S     = 17.0;  T_REST1_E     = 20.0;
T_PITCH_START = 20.0;
T_PITCH_HOLD_S= 22.0;  T_PITCH_HOLD_E= 27.0;
T_PITCH_END   = 29.0;
T_FINAL_S     = 29.0;  T_FINAL_E     = t(end);

%% ===== HAM TIEN ICH DARK THEME =====
function style_axes(ax, clr_ax, clr_txt, clr_grid)
    set(ax, 'Color', clr_ax, ...
            'XColor', clr_txt, 'YColor', clr_txt, ...
            'GridColor', clr_grid, 'MinorGridColor', clr_grid*1.5, ...
            'GridAlpha', 0.4, 'MinorGridAlpha', 0.2, ...
            'FontSize', 9, 'FontName', 'Consolas', ...
            'Box', 'on', 'LineWidth', 0.8);
    grid(ax, 'on'); grid(ax, 'minor');
end

%% ===== 3. FIGURE 1: TONG QUAN =====
fig1 = figure('Name','Bai 6 - Tong quan', ...
    'Position',[30 30 1350 860], 'Color', CLR_BG);

% ---------- ROLL ----------
ax1 = subplot(3,1,1);
patch([T_ROLL_HOLD_S T_ROLL_HOLD_E T_ROLL_HOLD_E T_ROLL_HOLD_S], ...
      [-12 -12 58 58], CLR_HOLD_R, 'EdgeColor','none', 'FaceAlpha',1);
hold on;
plot(t, Ra,  'Color',CLR_ACCEL, 'LineWidth',0.8, 'DisplayName','Accel');
plot(t, Rg,  'Color',CLR_GYRO,  'LineWidth',0.9, 'LineStyle','--', ...
    'DisplayName','Gyro (tich phan)');
plot(t, Rkf, 'Color',CLR_KF,    'LineWidth',2.2, 'DisplayName','Kalman Filter');
yline(45, 'Color',CLR_REF, 'LineWidth',1.2, 'LineStyle',':', ...
    'DisplayName','45° chuan');
yline(0,  'Color',CLR_REF, 'LineWidth',0.8, 'LineStyle',':');
ylabel('Goc (do)', 'Color',CLR_TXT, 'FontSize',10);
title('ROLL  –  xung quanh truc X', 'Color',CLR_TXT, 'FontSize',12, ...
    'FontWeight','bold');
lg = legend('Location','northeast', 'FontSize',8, 'TextColor',CLR_TXT);
lg.Color = [0.12 0.12 0.12]; lg.EdgeColor = CLR_GRID;
style_axes(ax1, CLR_AX, CLR_TXT, CLR_GRID);
xlim([0 t(end)]); ylim([-12 58]);
text(T_ROLL_HOLD_S+0.4, 50, '  Vung giu 45°', 'Color',CLR_REF, ...
    'FontSize',8, 'FontName','Consolas');

% ---------- PITCH ----------
ax2 = subplot(3,1,2);
patch([T_PITCH_HOLD_S T_PITCH_HOLD_E T_PITCH_HOLD_E T_PITCH_HOLD_S], ...
      [-12 -12 42 42], CLR_HOLD_P, 'EdgeColor','none', 'FaceAlpha',1);
hold on;
plot(t, Pa,  'Color',CLR_ACCEL, 'LineWidth',0.8, 'DisplayName','Accel');
plot(t, Pg,  'Color',CLR_GYRO,  'LineWidth',0.9, 'LineStyle','--', ...
    'DisplayName','Gyro (tich phan)');
plot(t, Pkf, 'Color',CLR_KF_P,  'LineWidth',2.2, 'DisplayName','Kalman Filter');
yline(30, 'Color',CLR_REF, 'LineWidth',1.2, 'LineStyle',':', ...
    'DisplayName','30° chuan');
yline(0,  'Color',CLR_REF, 'LineWidth',0.8, 'LineStyle',':');
ylabel('Goc (do)', 'Color',CLR_TXT, 'FontSize',10);
title('PITCH  –  xung quanh truc Y', 'Color',CLR_TXT, 'FontSize',12, ...
    'FontWeight','bold');
lg2 = legend('Location','northeast', 'FontSize',8, 'TextColor',CLR_TXT);
lg2.Color = [0.12 0.12 0.12]; lg2.EdgeColor = CLR_GRID;
style_axes(ax2, CLR_AX, CLR_TXT, CLR_GRID);
xlim([0 t(end)]); ylim([-12 42]);
text(T_PITCH_HOLD_S+0.4, 34, '  Vung giu 30°', 'Color',CLR_REF, ...
    'FontSize',8, 'FontName','Consolas');

% ---------- YAW ----------
ax3 = subplot(3,1,3);
plot(t, Yg, 'Color',CLR_GYRO, 'LineWidth',1.8, ...
    'DisplayName','Yaw – Gyro (tich phan Euler)');
hold on;
yline(0, 'Color',CLR_REF, 'LineWidth',0.8, 'LineStyle',':');
ylabel('Goc (do)', 'Color',CLR_TXT, 'FontSize',10);
xlabel('Thoi gian (s)', 'Color',CLR_TXT, 'FontSize',10);
title('YAW  –  Tich phan Gyroscope (Open-loop, khong can Magnetometer)', ...
    'Color',CLR_TXT, 'FontSize',12, 'FontWeight','bold');
lg3 = legend('Location','northwest', 'FontSize',8, 'TextColor',CLR_TXT);
lg3.Color = [0.12 0.12 0.12]; lg3.EdgeColor = CLR_GRID;
style_axes(ax3, CLR_AX, CLR_TXT, CLR_GRID);
xlim([0 t(end)]);

sgt = sgtitle('BAI 6  –  Uoc luong goc Roll / Pitch / Yaw  |  MPU6050', ...
    'Color',CLR_TXT, 'FontSize',14, 'FontWeight','bold');

saveas(fig1, 'bai6_overview.png');

%% ===== 4. PHAN TICH SO =====

% --- ROLL tai 45 do ---
mask_hold = (t >= T_ROLL_HOLD_S) & (t <= T_ROLL_HOLD_E);
Rhold     = Rkf(mask_hold);
mean_R    = mean(Rhold);
std_R     = std(Rhold);
err_R     = mean_R - 45;
maxdev_R  = max(abs(Rhold - 45));

% --- PITCH tai 30 do ---
mask_ph   = (t >= T_PITCH_HOLD_S) & (t <= T_PITCH_HOLD_E);
Phold     = Pkf(mask_ph);
mean_P    = mean(Phold);
std_P     = std(Phold);
err_P     = mean_P - 30;
maxdev_P  = max(abs(Phold - 30));

% --- Hoi tu Roll ---
mask_ret  = (t >= T_ROLL_END) & (t <= T_REST1_E);
t_ret     = t(mask_ret);
R_ret     = Rkf(mask_ret);
idx_cR    = find(abs(R_ret) < 1.0, 1, 'first');

% --- Hoi tu Pitch ---
mask_pret = (t >= T_PITCH_END) & (t <= T_FINAL_E);
t_pret    = t(mask_pret);
P_pret    = Pkf(mask_pret);
idx_cP    = find(abs(P_pret) < 1.0, 1, 'first');

% --- Latency ---
mask_up   = (t >= T_ROLL_START) & (t <= 6.0);
t_up      = t(mask_up);
idx_ac    = find(Ra(mask_up) > 5, 1, 'first');
idx_kf    = find(Rkf(mask_up) > 5, 1, 'first');

% --- Drift Yaw ---
mask_yaw  = (t >= T_FINAL_S) & (t <= T_FINAL_E);
t_ys      = t(mask_yaw);
Yg_s      = Yg(mask_yaw);
p_yg      = polyfit(t_ys, Yg_s, 1);
drift_dps = p_yg(1);

% In ket qua
fprintf('\n========================================\n');
fprintf('  PHAN TICH ROLL\n');
fprintf('========================================\n');
fprintf('\n[ON DINH tai Roll = 45 do  (t = 5 - 15 s)]\n');
fprintf('  Trung binh KF        : %.4f do\n', mean_R);
fprintf('  Sai so so voi 45 do  : %+.4f do\n', err_R);
fprintf('  STD (nhieu)          : %.4f do\n', std_R);
fprintf('  Sai so dinh          : %.4f do\n', maxdev_R);
if     maxdev_R < 0.5, fprintf('  => RAT TOT (< 0.5 do)\n');
elseif maxdev_R < 1.0, fprintf('  => TOT     (< 1.0 do)\n');
else,                  fprintf('  => CAN KIEM TRA\n'); end

fprintf('\n[HOI TU ve 0 sau khi tha Roll  (t = 17 - 20 s)]\n');
if ~isempty(idx_cR)
    tc = t_ret(idx_cR) - T_ROLL_END;
    fprintf('  Dat |Roll| < 1 do sau : %.2f s\n', tc);
    if tc<=2, fprintf('  => XUAT SAC (< 2s)\n');
    elseif tc<=3, fprintf('  => TOT (< 3s)\n');
    else, fprintf('  => CHAM (> 3s)\n'); end
else
    fprintf('  Gia tri cuoi Roll KF : %.4f do\n', R_ret(end));
end

fprintf('\n========================================\n');
fprintf('  PHAN TICH PITCH\n');
fprintf('========================================\n');
fprintf('\n[ON DINH tai Pitch = 30 do  (t = 22 - 27 s)]\n');
fprintf('  Trung binh KF        : %.4f do\n', mean_P);
fprintf('  Sai so so voi 30 do  : %+.4f do\n', err_P);
fprintf('  STD (nhieu)          : %.4f do\n', std_P);
fprintf('  Sai so dinh          : %.4f do\n', maxdev_P);
if     maxdev_P < 0.5, fprintf('  => RAT TOT (< 0.5 do)\n');
elseif maxdev_P < 1.0, fprintf('  => TOT     (< 1.0 do)\n');
else,                  fprintf('  => CAN KIEM TRA\n'); end

fprintf('\n[HOI TU ve 0 sau khi tha Pitch  (t = 29 - 35 s)]\n');
if ~isempty(idx_cP)
    tp = t_pret(idx_cP) - T_PITCH_END;
    fprintf('  Dat |Pitch| < 1 do sau : %.2f s\n', tp);
    if tp<=2, fprintf('  => XUAT SAC (< 2s)\n');
    elseif tp<=3, fprintf('  => TOT (< 3s)\n');
    else, fprintf('  => CHAM (> 3s)\n'); end
else
    fprintf('  Gia tri cuoi Pitch KF : %.4f do\n', P_pret(end));
end

fprintf('\n========================================\n');
fprintf('  DO TRE (LATENCY)\n');
fprintf('========================================\n');
if ~isempty(idx_ac) && ~isempty(idx_kf)
    lat = (t_up(idx_kf) - t_up(idx_ac)) * 1000;
    fprintf('  Accel vuot 5 do : t = %.3f s\n', t_up(idx_ac));
    fprintf('  KF    vuot 5 do : t = %.3f s\n', t_up(idx_kf));
    fprintf('  Do tre           : %.1f ms\n', lat);
    if abs(lat)<10, fprintf('  => RAT THAP (< 10ms)\n');
    elseif abs(lat)<50, fprintf('  => CHAP NHAN (< 50ms)\n');
    else, fprintf('  => CAO, xem lai Q/R\n'); end
end

fprintf('\n========================================\n');
fprintf('  DANH GIA YAW\n');
fprintf('========================================\n');
fprintf('  Drift rate : %.6f do/s  =  %.4f do/phut\n', drift_dps, drift_dps*60);
fprintf('  Tong drift : %.4f do / %.0f s\n', Yg_s(end)-Yg_s(1), t_ys(end)-t_ys(1));

fprintf('\n========================================\n');
fprintf('  SO SANH VOI GOC CHUAN\n');
fprintf('========================================\n');
fprintf('%-10s %-10s %-12s %-12s %-12s %-12s\n', ...
    'Chuan(do)','Canh','t_start','t_end','Mean_KF','Sai so');
fprintf('%s\n', repmat('-',1,68));

segs = {
    0,  'Roll',  T_STATIC_S,    T_STATIC_E,    Rkf;
    45, 'Roll',  T_ROLL_HOLD_S, T_ROLL_HOLD_E, Rkf;
    0,  'Roll',  T_REST1_S,     T_REST1_E,     Rkf;
    0,  'Pitch', T_STATIC_S,    T_STATIC_E,    Pkf;
    30, 'Pitch', T_PITCH_HOLD_S,T_PITCH_HOLD_E,Pkf;
    0,  'Pitch', T_FINAL_S,     T_FINAL_E,     Pkf;
};
for i = 1:size(segs,1)
    ref  = segs{i,1};  axis_ = segs{i,2};
    ts   = segs{i,3};  te    = segs{i,4};  sig = segs{i,5};
    mk   = (t>=ts) & (t<=te);
    v    = mean(sig(mk));  e = abs(v-ref);
    if e<0.5, g='Rat tot'; elseif e<1.0, g='Tot'; else, g='Can KT'; end
    fprintf('%-10d %-10s %-12.1f %-12.1f %-12.4f %-12.4f  %s\n', ...
        ref, axis_, ts, te, v, e, g);
end

%% ===== 5. FIGURE 2: PHAN TICH CHI TIET =====
fig2 = figure('Name','Bai 6 - Chi tiet', ...
    'Position',[50 50 1320 900], 'Color', CLR_BG);

% --- (a) Roll tang/giu/ve 0 ---
ax4 = subplot(2,2,1);
mask_a = (t >= T_ROLL_START-0.5) & (t <= T_REST1_E);
plot(t(mask_a), Ra(mask_a),  'Color',CLR_ACCEL, 'LineWidth',0.9, ...
    'DisplayName','Accel');
hold on;
plot(t(mask_a), Rkf(mask_a), 'Color',CLR_KF, 'LineWidth',2.2, ...
    'DisplayName','Kalman KF');
yline(45,'Color',CLR_REF,'LineWidth',1,'LineStyle','--');
yline(0, 'Color',CLR_REF,'LineWidth',0.8,'LineStyle','--');
xlabel('t (s)','Color',CLR_TXT); ylabel('Roll (do)','Color',CLR_TXT);
title('Roll: Tang – Giu – Ve 0','Color',CLR_TXT,'FontWeight','bold');
lg4 = legend('FontSize',8,'TextColor',CLR_TXT);
lg4.Color=[0.12 0.12 0.12]; lg4.EdgeColor=CLR_GRID;
style_axes(ax4, CLR_AX, CLR_TXT, CLR_GRID);

% --- (b) Pitch tang/giu/ve 0 ---
ax5 = subplot(2,2,2);
mask_b = (t >= T_PITCH_START-0.5) & (t <= T_FINAL_E);
plot(t(mask_b), Pa(mask_b),  'Color',CLR_ACCEL, 'LineWidth',0.9, ...
    'DisplayName','Accel');
hold on;
plot(t(mask_b), Pkf(mask_b), 'Color',CLR_KF_P, 'LineWidth',2.2, ...
    'DisplayName','Kalman KF');
yline(30,'Color',CLR_REF,'LineWidth',1,'LineStyle','--');
yline(0, 'Color',CLR_REF,'LineWidth',0.8,'LineStyle','--');
xlabel('t (s)','Color',CLR_TXT); ylabel('Pitch (do)','Color',CLR_TXT);
title('Pitch: Tang – Giu – Ve 0','Color',CLR_TXT,'FontWeight','bold');
lg5 = legend('FontSize',8,'TextColor',CLR_TXT);
lg5.Color=[0.12 0.12 0.12]; lg5.EdgeColor=CLR_GRID;
style_axes(ax5, CLR_AX, CLR_TXT, CLR_GRID);

% --- (c) Nhieu tai goc giu ---
ax6 = subplot(2,2,3);
t_hn = t(mask_hold) - T_ROLL_HOLD_S;
dev_R = Rhold - 45;
fill([t_hn; flipud(t_hn)], ...
     [ones(size(t_hn))*0.5; ones(size(t_hn))*(-0.5)], ...
     [0.2 0.2 0.1], 'EdgeColor','none', 'FaceAlpha',0.5);
hold on;
plot(t_hn, dev_R, 'Color',CLR_KF, 'LineWidth',1.5);
yline(0.5,  'Color',CLR_REF,'LineWidth',1,'LineStyle','--');
yline(-0.5, 'Color',CLR_REF,'LineWidth',1,'LineStyle','--');
yline(0,    'Color',[0.4 1 0.4],'LineWidth',1.2);
xlabel('Thoi gian giu goc (s)','Color',CLR_TXT);
ylabel('Sai so (do)','Color',CLR_TXT);
title(sprintf('Nhieu Roll tai 45°  |  STD=%.4f°  |  Max=%.4f°', ...
    std_R, maxdev_R), 'Color',CLR_TXT, 'FontWeight','bold');
style_axes(ax6, CLR_AX, CLR_TXT, CLR_GRID);

% --- (d) Drift Yaw ---
ax7 = subplot(2,2,4);
t_fit = linspace(t_ys(1), t_ys(end), 300);
plot(t_ys, Yg_s, 'Color',CLR_GYRO, 'LineWidth',1.8, ...
    'DisplayName','Yaw do');
hold on;
plot(t_fit, polyval(p_yg,t_fit), 'Color',CLR_WARN, 'LineWidth',1.5, ...
    'LineStyle','--', 'DisplayName', ...
    sprintf('Xu huong (%.6f°/s)', drift_dps));
xlabel('t (s)','Color',CLR_TXT); ylabel('Yaw (do)','Color',CLR_TXT);
title('Drift Yaw – Kiem chung do on dinh', ...
    'Color',CLR_TXT,'FontWeight','bold');
lg7 = legend('FontSize',8,'TextColor',CLR_TXT);
lg7.Color=[0.12 0.12 0.12]; lg7.EdgeColor=CLR_GRID;
style_axes(ax7, CLR_AX, CLR_TXT, CLR_GRID);

sgtitle('BAI 6  –  Phan tich chi tiet', ...
    'Color',CLR_TXT,'FontSize',14,'FontWeight','bold');
saveas(fig2, 'bai6_detail.png');

%% ===== 6. FIGURE 3: HIEU QUA LOC NHIEU =====
fig3 = figure('Name','Bai 6 - Hieu qua loc nhieu', ...
    'Position',[70 70 1280 440], 'Color', CLR_BG);

% --- Roll ---
ax8 = subplot(1,2,1);
nRa  = Ra(mask_hold)  - mean(Ra(mask_hold));
nRkf = Rkf(mask_hold) - mean(Rkf(mask_hold));
tn   = t(mask_hold) - T_ROLL_HOLD_S;
plot(tn, nRa,  'Color',CLR_ACCEL, 'LineWidth',0.9, ...
    'DisplayName', sprintf('Accel  STD=%.4f°', std(nRa)));
hold on;
plot(tn, nRkf, 'Color',CLR_KF,    'LineWidth',1.8, ...
    'DisplayName', sprintf('KF     STD=%.4f°', std(nRkf)));
xlabel('Thoi gian giu goc (s)','Color',CLR_TXT);
ylabel('\Delta goc (do)','Color',CLR_TXT);
title('Nhieu Roll = 45°  (Accel vs KF)','Color',CLR_TXT,'FontWeight','bold');
lg8 = legend('FontSize',9,'TextColor',CLR_TXT);
lg8.Color=[0.12 0.12 0.12]; lg8.EdgeColor=CLR_GRID;
style_axes(ax8, CLR_AX, CLR_TXT, CLR_GRID);
ratio_R = std(nRa)/std(nRkf);
text(0.03, 0.91, sprintf('KF giam nhieu %.1fx', ratio_R), ...
    'Units','normalized','Color',CLR_KF,'FontSize',10,'FontWeight','bold');

% --- Pitch ---
ax9 = subplot(1,2,2);
nPa  = Pa(mask_ph)  - mean(Pa(mask_ph));
nPkf = Pkf(mask_ph) - mean(Pkf(mask_ph));
tp2  = t(mask_ph) - T_PITCH_HOLD_S;
plot(tp2, nPa,  'Color',CLR_ACCEL, 'LineWidth',0.9, ...
    'DisplayName', sprintf('Accel  STD=%.4f°', std(nPa)));
hold on;
plot(tp2, nPkf, 'Color',CLR_KF_P,  'LineWidth',1.8, ...
    'DisplayName', sprintf('KF     STD=%.4f°', std(nPkf)));
xlabel('Thoi gian giu goc (s)','Color',CLR_TXT);
ylabel('\Delta goc (do)','Color',CLR_TXT);
title('Nhieu Pitch = 30°  (Accel vs KF)','Color',CLR_TXT,'FontWeight','bold');
lg9 = legend('FontSize',9,'TextColor',CLR_TXT);
lg9.Color=[0.12 0.12 0.12]; lg9.EdgeColor=CLR_GRID;
style_axes(ax9, CLR_AX, CLR_TXT, CLR_GRID);
ratio_P = std(nPa)/std(nPkf);
text(0.03, 0.91, sprintf('KF giam nhieu %.1fx', ratio_P), ...
    'Units','normalized','Color',CLR_KF_P,'FontSize',10,'FontWeight','bold');

sgtitle('BAI 6  –  Hieu qua loc nhieu cua Kalman Filter', ...
    'Color',CLR_TXT,'FontSize',13,'FontWeight','bold');
saveas(fig3, 'bai6_noise.png');

%% ===== 7. TOM TAT CUOI =====
fprintf('\n========================================\n');
fprintf('  TOM TAT KET QUA BAI 6\n');
fprintf('========================================\n');
fprintf('  ROLL  – Sai so:%.4f do | STD:%.4f do | Giam nhieu:%.1fx\n', ...
    err_R, std_R, ratio_R);
fprintf('  PITCH – Sai so:%.4f do | STD:%.4f do | Giam nhieu:%.1fx\n', ...
    err_P, std_P, ratio_P);
fprintf('  YAW   – Drift: %.6f do/s  (%.4f do/phut)\n', drift_dps, drift_dps*60);
if maxdev_R<0.5 && maxdev_P<0.5
    fprintf('\n  => KET LUAN: DAT CHUAN RAT TOT (sai so < 0.5 do)\n');
elseif maxdev_R<1.0 && maxdev_P<1.0
    fprintf('\n  => KET LUAN: DAT CHUAN TOT (sai so < 1.0 do)\n');
else
    fprintf('\n  => KET LUAN: CAN KIEM TRA LAI (sai so > 1.0 do)\n');
end
fprintf('\n  File da luu:\n');
fprintf('    bai6_overview.png\n');
fprintf('    bai6_detail.png\n');
fprintf('    bai6_noise.png\n');
fprintf('\n=== HOAN TAT BAI 6 ===\n');