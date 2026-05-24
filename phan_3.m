clc; clear; close all;
angles_list = [0, 15, 30, 45, 60, 90];
fid = fopen('bai3_data.txt', 'r');
if fid == -1
    error('Khong mo duoc file bai3_data.txt. Kiem tra lai duong dan.');
end

raw = [];
while ~feof(fid)
    line = strtrim(fgetl(fid));
    parts = strsplit(line, ',');
    if numel(parts) == 4
        vals = str2double(parts);
        if ~any(isnan(vals))
            raw = [raw; vals]; %#ok<AGROW>
        end
    end
end
fclose(fid);

if isempty(raw)
    error('Khong doc duoc du lieu. Kiem tra lai dinh dang file.');
end
fprintf('Doc duoc %d dong du lieu tu file.\n', size(raw, 1));

%% ---- 2. TINH TRUNG BINH THEO GOC ----
ax_meas = zeros(1,6);
ay_meas = zeros(1,6);
az_meas = zeros(1,6);

for i = 1:6
    ang = angles_list(i);
    idx = abs(raw(:,1) - ang) < 0.5; 
    if sum(idx) == 0
        warning('Khong co du lieu cho goc %d do!', ang);
        continue
    end
    ax_meas(i) = mean(raw(idx, 2));
    ay_meas(i) = mean(raw(idx, 3));
    az_meas(i) = mean(raw(idx, 4));
end

%% ---- 3. GIA TRI LY THUYET ----
angles_rad = deg2rad(angles_list);
ax_theory  = sin(angles_rad);
ay_theory  = zeros(1,6);
az_theory  = cos(angles_rad);

%% ---- 4. TINH |a| VA SAI LECH ----
mag_meas = sqrt(ax_meas.^2 + ay_meas.^2 + az_meas.^2);
err_pct  = abs(mag_meas - 1.0) * 100;

fprintf('\n=== BAI 3: ANH HUONG GIA TOC TRONG TRUONG ===\n\n');
fprintf('%-6s | %-8s %-8s %-8s | %-8s %-8s %-8s | %-6s | %-8s\n', ...
    'Goc','Ax_LT','Ay_LT','Az_LT','Ax_do','Ay_do','Az_do','|a|','Sai lech');
fprintf('%s\n', repmat('-',1,80));
for i = 1:6
    fprintf('%-6d | %-8.3f %-8.3f %-8.3f | %-8.3f %-8.3f %-8.3f | %-6.4f | %.2f%%\n', ...
        angles_list(i), ...
        ax_theory(i), ay_theory(i), az_theory(i), ...
        ax_meas(i),   ay_meas(i),   az_meas(i), ...
        mag_meas(i),  err_pct(i));
end

fprintf('\n--- DANH GIA ---\n');
if all(err_pct <= 3.0)
    fprintf('[OK] Tat ca goc do: sai lech |a| < 3%%\n');
else
    bad = angles_list(err_pct > 3.0);
    fprintf('[!] Goc do lech >3%%: %s -> Can hieu chinh gain Accel\n', mat2str(bad));
end

%% ---- 5. DARK THEME SETUP ----
BG   = [0.10 0.10 0.10];
AX   = [0.15 0.15 0.15];
FG   = [1.00 1.00 1.00];
GRID = [0.35 0.35 0.35];
CLT  = [0.95 0.95 0.95];
CRX  = [1.00 0.35 0.35];
CGR  = [0.35 0.90 0.35];
CBL  = [0.35 0.65 1.00];
COR  = [1.00 0.70 0.10];
CGB  = [0.50 0.80 1.00];
fig = figure('Name','Bai 3 - Anh huong gia toc trong truong', ...
             'Position',[50 50 1300 800], ...
             'Color', BG); 
    function style_ax(ax, AX, FG, GRID)
        set(ax, 'Color', AX, ...
                'XColor', FG, 'YColor', FG, 'ZColor', FG, ...
                'GridColor', GRID, 'MinorGridColor', GRID, ...
                'GridAlpha', 0.5);
        ax.Title.Color  = FG;
        ax.XLabel.Color = FG;
        ax.YLabel.Color = FG;
        lg = ax.Legend;
        if ~isempty(lg)
            lg.Color     = [0.18 0.18 0.18];
            lg.TextColor = FG;
            lg.EdgeColor = [0.5 0.5 0.5];
        end
    end

%% ---- 6. VE DO THI ----
% --- Ax ---
ax1 = subplot(2,3,1);
plot(angles_list, ax_theory, '--o', 'Color', CLT, 'LineWidth',1.5,'MarkerSize',8, ...
     'DisplayName','Ly thuyet'); hold on;
plot(angles_list, ax_meas,   '-s',  'Color', CRX, 'LineWidth',1.5,'MarkerSize',8, ...
     'DisplayName','Do thuc te');
xlabel('Goc nghieng (do)'); ylabel('Ax (g)');
title('Ax = sin(\theta)'); legend('Location','best'); grid on;
style_ax(ax1, AX, FG, GRID);
% --- Ay ---
ax2 = subplot(2,3,2);
plot(angles_list, ay_theory, '--o', 'Color', CLT, 'LineWidth',1.5,'MarkerSize',8, ...
     'DisplayName','Ly thuyet'); hold on;
plot(angles_list, ay_meas,   '-s',  'Color', CGR, 'LineWidth',1.5,'MarkerSize',8, ...
     'DisplayName','Do thuc te');
xlabel('Goc nghieng (do)'); ylabel('Ay (g)');
title('Ay \approx 0'); legend('Location','best'); grid on;
style_ax(ax2, AX, FG, GRID);

% --- Az ---
ax3 = subplot(2,3,3);
plot(angles_list, az_theory, '--o', 'Color', CLT, 'LineWidth',1.5,'MarkerSize',8, ...
     'DisplayName','Ly thuyet'); hold on;
plot(angles_list, az_meas,   '-s',  'Color', CBL, 'LineWidth',1.5,'MarkerSize',8, ...
     'DisplayName','Do thuc te');
xlabel('Goc nghieng (do)'); ylabel('Az (g)');
title('Az = cos(\theta)'); legend('Location','best'); grid on;
style_ax(ax3, AX, FG, GRID);
% --- |a| tong hop ---
ax4 = subplot(2,3,4);
plot(angles_list, ones(1,6), '--', 'Color', CLT, 'LineWidth',2, ...
     'DisplayName','1g ly tuong'); hold on;
plot(angles_list, mag_meas, '-o', 'Color', COR, 'LineWidth',1.5,'MarkerSize',8, ...
     'DisplayName','|a| do duoc');
fill([angles_list, fliplr(angles_list)], ...
     [ones(1,6)*0.97, ones(1,6)*1.03], ...
     [0.3 0.5 0.8], 'FaceAlpha',0.25, 'EdgeColor','none', ...
     'DisplayName','±3%');
xlabel('Goc nghieng (do)'); ylabel('|a| (g)');
title('Gia toc tong hop |a| (phai = 1g)');
legend('Location','best'); grid on; ylim([0.90 1.10]);
style_ax(ax4, AX, FG, GRID);

% --- Sai lech % ---
ax5 = subplot(2,3,5);
b = bar(angles_list, err_pct, 'FaceColor', [0.30 0.60 1.00], 'EdgeColor','none');
hold on;
yline(3.0, '--', '3% gioi han', 'Color', [1 0.4 0.4], 'LineWidth',1.5, ...
      'LabelVerticalAlignment','bottom', 'FontSize', 9);
xlabel('Goc nghieng (do)'); ylabel('Sai lech |a| (%)');
title('Sai lech so voi 1g ly tuong'); grid on;
ylim([0, max(max(err_pct), 3)*1.4]);
style_ax(ax5, AX, FG, GRID);
ax5.YAxis.Color = FG;

% --- Vector gia toc 2D ---
ax6 = subplot(2,3,6);
theta_cont = linspace(0, pi/2, 100);
plot(sin(theta_cont), cos(theta_cont), '--', 'Color', CLT, 'LineWidth',1.5, ...
     'DisplayName','Duong tron don vi ly tuong'); hold on;
scatter(ax_meas, az_meas, 80, CGB, 'filled', 'DisplayName','Do thuc te');
for i = 1:6
    text(ax_meas(i)+0.015, az_meas(i)+0.015, sprintf('%d°', angles_list(i)), ...
         'FontSize',9, 'Color', FG);
end
xlabel('Ax (g)'); ylabel('Az (g)');
title('Quy dao vector gia toc (Ax vs Az)');
legend('Location','best'); grid on; axis equal;
xlim([-0.1 1.1]); ylim([-0.1 1.1]);
style_ax(ax6, AX, FG, GRID);
% --- Super title ---
sgt = sgtitle('BAI 3 - KHAO SAT ANH HUONG GIA TOC TRONG TRUONG', ...
              'FontSize',13, 'FontWeight','bold');
sgt.Color = FG;

saveas(fig, 'bai3_gravity.png');
fprintf('\n[OK] Da luu bai3_gravity.png\n');