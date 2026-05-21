%% ================================================================
%  MPU6500_Lab.m
%  PHAN TICH DU LIEU - KY THUAT CAM BIEN
%  Cam bien IMU MPU6500 - Day du 7 bai thuc hanh
% ================================================================
%
%  CAC H DAT TEN FILE DU LIEU (xuat tu Arduino Serial Monitor):
%    bai1_data.csv  - Bai 1: 500 mau tinh (6 cot: ax,ay,az,gx,gy,gz)
%    bai3_data.csv  - Bai 3: Do thi goc nghieng (7 cot: angle,ax,ay,az,gx,gy,gz)
%    bai4_data.csv  - Bai 4: Bo loc (7 cot: time_ms,roll_accel,pitch_accel,roll_cf,pitch_cf,roll_kf,pitch_kf)
%    bai5_shock.csv - Bai 5A: Shock (6 cot: time_ms,ax,ay,az,mag,shock_flag)
%    bai5_vib.txt   - Bai 5B: Dao dong (1 cot: gia tri |a|-1g, 512 dong)
%    bai6_data.csv  - Bai 6: Goc (9 cot: time_ms,roll_a,pitch_a,roll_g,pitch_g,yaw_g,roll_kf,pitch_kf,yaw_kf)
%    bai7_data.csv  - Bai 7: Tong hop (7 cot: time_ms,roll_kf,pitch_kf,yaw_gyro,temp_c,accel_mag,shock)
%
%  HUONG DAN XUAT DU LIEU TU ARDUINO:
%    1. Mo Serial Monitor (115200 baud)
%    2. Chon bai tuong ung, copy toan bo du lieu CSV
%    3. Dan vao Notepad, luu thanh file .csv (xoa dong header neu co)
%    4. Dat file cung thu muc voi MPU6500_Lab.m
%    5. Chay: MPU6500_Lab
%
%  YEU CAU TOOLBOX:
%    - Signal Processing Toolbox (butter, filtfilt, fft)
%    - Statistics and Machine Learning Toolbox (std, mean - co san)
%
%  Tuong thich: MATLAB R2019b tro len
% ================================================================

function MPU6500_Lab()
    clc; close all;
    
    fprintf('========================================\n');
    fprintf('  MPU6500 - KY THUAT CAM BIEN\n');
    fprintf('  Phan tich du lieu - Day du 7 bai\n');
    fprintf('========================================\n\n');
    
    while true
        fprintf('--- MENU CHINH ---\n');
        fprintf('  1 - Bai 1 : Do luong co ban & Dac tinh tinh\n');
        fprintf('  2 - Bai 2 : Hieu chinh Offset (Calibration)\n');
        fprintf('  3 - Bai 3 : Anh huong gia toc trong truong\n');
        fprintf('  4 - Bai 4 : Bo loc (Complementary & Kalman)\n');
        fprintf('  5 - Bai 5 : Do shock & Phan tich FFT\n');
        fprintf('  6 - Bai 6 : Uoc luong goc Roll/Pitch/Yaw\n');
        fprintf('  7 - Bai 7 : Bai tap tong hop (Dashboard)\n');
        fprintf('  0 - Thoat\n');
        fprintf('----------------------------------------\n');
        
        choice = input('Chon bai (0-7): ', 's');
        choice = strtrim(choice);
        
        switch choice
            case '1', task1_static();
            case '2', MPU6500_Bai2_Calibration();
            case '3', task3_gravity();
            case '4', task4_filter();
            case '5', task5_shock_fft();
            case '6', task6_angles();
            case '7', task7_summary();
            case '0'
                fprintf('\nThoat chuong trinh.\n');
                return;
            otherwise
                fprintf('[!] Lua chon khong hop le.\n\n');
        end
    end
end

%% ================================================================
%  BAI 1: DO LUONG CO BAN & DAC TINH TINH
%  File: bai1_data.csv (500 dong x 6 cot: ax,ay,az,gx,gy,gz)
% ================================================================
function task1_static()
    fprintf('\n========================================\n');
    fprintf('  BAI 1: DO LUONG CO BAN & DAC TINH TINH\n');
    fprintf('========================================\n');
    fprintf('File can: bai1_data.csv\n');
    fprintf('Dinh dang: ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps\n');
    fprintf('(500 dong, cam bien nam yen)\n\n');

    fname = input('Nhap ten file (Enter = bai1_data.csv): ', 's');
    if isempty(fname), fname = 'bai1_data.csv'; end

    data = load_csv(fname, 6);
    if isempty(data), return; end
    if size(data,1) < 10
        fprintf('[!] Can it nhat 10 mau, chi co %d dong.\n', size(data,1));
        return;
    end

    ax = data(:,1); ay = data(:,2); az = data(:,3);
    gx = data(:,4); gy = data(:,5); gz = data(:,6);
    N  = length(ax);
    t  = (0:N-1) * 0.01;   % 100 Hz -> dt = 10ms

    % --- Thong ke ---
    m_ax = mean(ax); m_ay = mean(ay); m_az = mean(az);
    m_gx = mean(gx); m_gy = mean(gy); m_gz = mean(gz);
    s_ax = std(ax);  s_ay = std(ay);  s_az = std(az);
    s_gx = std(gx);  s_gy = std(gy);  s_gz = std(gz);
    mag  = mean(sqrt(ax.^2 + ay.^2 + az.^2));
    err_pct = abs(mag - 1.0) * 100;

    fprintf('\n--- KET QUA DAC TINH TINH (%d mau) ---\n', N);
    fprintf('\n[ACCELEROMETER] (don vi: g)\n');
    fprintf('         |   Ax      |   Ay      |   Az\n');
    fprintf('---------|-----------|-----------|----------\n');
    fprintf('Mean (g) | %+8.5f  | %+8.5f  | %+8.5f\n', m_ax, m_ay, m_az);
    fprintf('STD  (g) |  %8.6f |  %8.6f |  %8.6f\n', s_ax, s_ay, s_az);
    fprintf('\n[GYROSCOPE] (don vi: deg/s)\n');
    fprintf('           |    Gx     |    Gy     |    Gz\n');
    fprintf('-----------|-----------|-----------|----------\n');
    fprintf('Mean(d/s)  | %+8.5f  | %+8.5f  | %+8.5f\n', m_gx, m_gy, m_gz);
    fprintf('STD (d/s)  |  %8.6f |  %8.6f |  %8.6f\n', s_gx, s_gy, s_gz);
    fprintf('\n[DANH GIA]\n');
    fprintf('  |a| trung binh = %.5f g  (ly tuong: 1.000)\n', mag);
    fprintf('  Sai lech = %.2f%%\n', err_pct);
    if err_pct < 3,  fprintf('  [OK] |a| trong gioi han 3%%\n');
    else,            fprintf('  [!] |a| lech > 3%% -> can hieu chinh gain\n'); end
    if abs(m_gx)<0.05 && abs(m_gy)<0.05 && abs(m_gz)<0.05
        fprintf('  [OK] Gyro offset < 0.05 deg/s\n');
    else
        fprintf('  [!] Gyro offset lon -> can hieu chinh (Bai 2)\n');
    end

    % --- Drift gyro: tich phan theo thoi gian ---
    drift_x = cumsum(gx) * 0.01;
    drift_y = cumsum(gy) * 0.01;
    drift_z = cumsum(gz) * 0.01;

    % --- Ve do thi ---
    fig = figure('Name','Bai 1 - Dac tinh tinh','NumberTitle','off','Position',[100 100 1400 900]);
    sgtitle('Bai 1: Do luong co ban & Dac tinh tinh','FontSize',14,'FontWeight','bold');

    % Tin hieu accel theo thoi gian
    subplot(3,3,1);
    plot(t,ax,'r',t,ay,'g',t,az,'b','LineWidth',1);
    xlabel('Thoi gian (s)'); ylabel('Gia toc (g)');
    title('Gia toc theo thoi gian'); legend('Ax','Ay','Az'); grid on;

    % Tin hieu gyro theo thoi gian
    subplot(3,3,2);
    plot(t,gx,'r',t,gy,'g',t,gz,'b','LineWidth',1);
    xlabel('Thoi gian (s)'); ylabel('Toc do quay (deg/s)');
    title('Gyro theo thoi gian'); legend('Gx','Gy','Gz'); grid on;

    % Histogram Ax
    subplot(3,3,3);
    histogram(ax, 30, 'FaceColor','#e63946','EdgeColor','white');
    xline(m_ax,'y--','LineWidth',2,'Label',sprintf('Mean=%.4f',m_ax));
    xlabel('Ax (g)'); ylabel('So mau');
    title(sprintf('Phan phoi Ax | STD=%.5f',s_ax)); grid on;

    % Histogram Ay
    subplot(3,3,4);
    histogram(ay, 30, 'FaceColor','#2a9d8f','EdgeColor','white');
    xline(m_ay,'y--','LineWidth',2,'Label',sprintf('Mean=%.4f',m_ay));
    xlabel('Ay (g)'); ylabel('So mau');
    title(sprintf('Phan phoi Ay | STD=%.5f',s_ay)); grid on;

    % Histogram Az
    subplot(3,3,5);
    histogram(az, 30, 'FaceColor','#457b9d','EdgeColor','white');
    xline(m_az,'y--','LineWidth',2,'Label',sprintf('Mean=%.4f',m_az));
    xlabel('Az (g)'); ylabel('So mau');
    title(sprintf('Phan phoi Az | STD=%.5f',s_az)); grid on;

    % Histogram Gyro Gz
    subplot(3,3,6);
    histogram(gz, 30, 'FaceColor','#f4a261','EdgeColor','white');
    xline(m_gz,'y--','LineWidth',2,'Label',sprintf('Mean=%.4f',m_gz));
    xlabel('Gz (deg/s)'); ylabel('So mau');
    title(sprintf('Phan phoi Gz | STD=%.5f',s_gz)); grid on;

    % Drift gyro
    subplot(3,3,7);
    plot(t,drift_x,'r',t,drift_y,'g',t,drift_z,'b','LineWidth',1.5);
    xlabel('Thoi gian (s)'); ylabel('Goc tich phan (deg)');
    title('Gyro Drift (tich phan theo t)'); legend('X','Y','Z'); grid on;

    % |a| magnitude
    mag_all = sqrt(ax.^2+ay.^2+az.^2);
    subplot(3,3,8);
    plot(t, mag_all, 'y', 'LineWidth', 1);
    yline(1.0,'r--','LineWidth',1.5,'Label','Ly tuong 1g');
    yline(1.03,'b:','LineWidth',1); yline(0.97,'b:','LineWidth',1,'Label','±3%');
    xlabel('Thoi gian (s)'); ylabel('|a| (g)');
    title(sprintf('Bien do gia toc tong |a| trung binh=%.4f g',mag)); grid on;

    % Bang tong ket
    subplot(3,3,9);
    axis off;
    entries = {
        'N mau', num2str(N);
        'Mean Ax (g)', sprintf('%+.5f',m_ax);
        'Mean Ay (g)', sprintf('%+.5f',m_ay);
        'Mean Az (g)', sprintf('%+.5f',m_az);
        'STD Ax',  sprintf('%.6f',s_ax);
        'STD Ay',  sprintf('%.6f',s_ay);
        'STD Az',  sprintf('%.6f',s_az);
        'Mean Gx (d/s)', sprintf('%+.5f',m_gx);
        'Mean Gy (d/s)', sprintf('%+.5f',m_gy);
        'Mean Gz (d/s)', sprintf('%+.5f',m_gz);
        'STD Gx', sprintf('%.6f',s_gx);
        '|a| TB', sprintf('%.5f g',mag);
        'Sai lech |a|', sprintf('%.2f%%',err_pct);
    };
    t_tbl = uitable(fig,'Data',entries,'ColumnName',{'Thong so','Gia tri'},...
        'ColumnWidth',{130,100},'Units','normalized',...
        'Position',[0.68 0.03 0.30 0.28],'FontSize',8);
    save_figure(fig,'bai1_result.png');
    fprintf('\n[OK] Bai 1 hoan tat.');
end

%% ================================================================
%  MPU6500_Bai2_Calibration.m
%  BAI 2A: Hieu chinh Gyroscope   -> bai2a_result.png
%  BAI 2B: Hieu chinh Accel       -> bai2b_result.png
%
%  SUA LOI:
%   - Parser bo qua du lieu truoc header "Pos N -" dau tien
%   - Canh bao Gyro dua tren STD (nhieu), khong phai gia tri offset
% ================================================================
function MPU6500_Bai2_Calibration()
    clc; close all;

    fname_g = input('File gyro  (Enter = bai2a_data.txt): ','s');
    if isempty(fname_g), fname_g = 'bai2a_data.txt'; end
    fname_a = input('File accel (Enter = bai2b_data.txt): ','s');
    if isempty(fname_a), fname_a = 'bai2b_data.txt'; end

    dg = doc_csv(fname_g, 3);
    [pos_data, pos_labels, n_pos] = doc_6pos(fname_a);
    if isempty(dg) || n_pos == 0, return; end

    %% ============================================================
    %  TINH TOAN GYRO
    % ============================================================
    gx = dg(:,1); gy = dg(:,2); gz = dg(:,3);
    N  = length(gx);
    t  = (0:N-1)*0.01;

    gx_off = mean(gx); gy_off = mean(gy); gz_off = mean(gz);
    std_gx = std(gx);  std_gy = std(gy);  std_gz = std(gz);

    gx_c = gx - gx_off;
    gy_c = gy - gy_off;
    gz_c = gz - gz_off;

    drift_raw = cumsum(gy)   * 0.01;
    drift_cal = cumsum(gy_c) * 0.01;

    %% ============================================================
    %  TINH TOAN ACCEL (6-POSITION)
    % ============================================================
    mAx = zeros(n_pos,1); mAy = zeros(n_pos,1); mAz = zeros(n_pos,1);
    for p = 1:n_pos
        d = pos_data{p};
        mAx(p) = mean(d(:,1));
        mAy(p) = mean(d(:,2));
        mAz(p) = mean(d(:,3));
    end

    %  Pos 1=Zup, 2=Zdn, 3=Xup, 4=Xdn, 5=Yup, 6=Ydn
    az_off = -(mAz(1) + mAz(2)) / 2;
    ax_off = -(mAx(3) + mAx(4)) / 2;
    ay_off = -(mAy(5) + mAy(6)) / 2;

    mR = zeros(n_pos,1); mC = zeros(n_pos,1);
    eR = zeros(n_pos,1); eC = zeros(n_pos,1);
    for p = 1:n_pos
        d = pos_data{p};
        mR(p) = mean(sqrt(d(:,1).^2 + d(:,2).^2 + d(:,3).^2));
        mC(p) = mean(sqrt((d(:,1)+ax_off).^2 + ...
                          (d(:,2)+ay_off).^2 + ...
                          (d(:,3)+az_off).^2));
        eR(p) = (mR(p)-1)*100;
        eC(p) = (mC(p)-1)*100;
    end
    maxErrCal = max(abs(eC));

    %% ============================================================
    %  CONSOLE
    % ============================================================
    fprintf('\n=== BAI 2A: GYRO ===\n');
    fprintf('  Gx: offset=%+.5f  STD=%.5f deg/s  -> %s\n', gx_off, std_gx, ok(std_gx<0.05));
    fprintf('  Gy: offset=%+.5f  STD=%.5f deg/s  -> %s\n', gy_off, std_gy, ok(std_gy<0.05));
    fprintf('  Gz: offset=%+.5f  STD=%.5f deg/s  -> %s\n', gz_off, std_gz, ok(std_gz<0.05));

    fprintf('\n=== BAI 2B: ACCEL ===\n');
    fprintf('  ax_off=%+.5f g  ay_off=%+.5f g  az_off=%+.5f g\n', ax_off,ay_off,az_off);
    fprintf('\n  %-14s  |a|Raw  Err%%   |a|Cal  Err%%\n','Vi tri');
    for p = 1:n_pos
        fprintf('  %-14s  %.4f  %+.2f%%  %.4f  %+.2f%%\n', ...
            pos_labels{p}, mR(p),eR(p), mC(p),eC(p));
    end
    fprintf('  Max err sau calib: %.3f%%  -> %s\n', maxErrCal, ok(maxErrCal<3));

    %% ============================================================
    %  FIGURE 1 - BAI 2A  (2 x 2)
    % ============================================================
    fig1 = figure('Name','Bai 2A - Gyro','NumberTitle','off', ...
                  'Position',[40 80 1100 680]);
    sgtitle('Bai 2A: Hieu chinh Gyroscope', 'FontSize',14,'FontWeight','bold');

    % [1] Gyro Raw
    subplot(2,2,1);
    plot(t,gx,'r','LineWidth',0.8,'DisplayName', ...
        sprintf('Gx  offset=%+.4f',gx_off)); hold on;
    plot(t,gy,'g','LineWidth',0.8,'DisplayName', ...
        sprintf('Gy  offset=%+.4f',gy_off));
    plot(t,gz,'b','LineWidth',0.8,'DisplayName', ...
        sprintf('Gz  offset=%+.4f',gz_off));
    plot([t(1) t(end)],[0 0],'k--','LineWidth',1,'HandleVisibility','off');
    hold off;
    xlabel('Thoi gian (s)'); ylabel('deg/s');
    title('Gyro tho (Raw)  —  cam bien dung yen');
    legend('Location','best','FontSize',8);
    grid on;

    % [2] Gyro sau hieu chinh
    subplot(2,2,2);
    plot(t,gx_c,'r','LineWidth',0.8,'DisplayName', ...
        sprintf('Gx  STD=%.4f',std_gx)); hold on;
    plot(t,gy_c,'g','LineWidth',0.8,'DisplayName', ...
        sprintf('Gy  STD=%.4f',std_gy));
    plot(t,gz_c,'b','LineWidth',0.8,'DisplayName', ...
        sprintf('Gz  STD=%.4f',std_gz));
    plot([t(1) t(end)],[  0    0  ],'k--','LineWidth',1.2,'HandleVisibility','off');
    plot([t(1) t(end)],[ 0.05  0.05],'k:','LineWidth',1,  'HandleVisibility','off');
    plot([t(1) t(end)],[-0.05 -0.05],'k:','LineWidth',1,  'HandleVisibility','off');
    hold off;
    xlabel('Thoi gian (s)'); ylabel('deg/s');
    title('Gyro sau hieu chinh  (duong cham = +-0.05 deg/s)');
    legend('Location','best','FontSize',8);
    grid on; ylim([-0.20 0.20]);

    % [3] Drift tich phan Gy
    subplot(2,2,3);
    plot(t, drift_raw,'r--','LineWidth',2, ...
        'DisplayName',sprintf('Gy raw  drift cuoi=%.3f deg',drift_raw(end))); hold on;
    plot(t, drift_cal,'g-', 'LineWidth',2, ...
        'DisplayName',sprintf('Gy sau calib  drift cuoi=%.5f deg',drift_cal(end)));
    plot([t(1) t(end)],[0 0],'k--','LineWidth',1,'HandleVisibility','off');
    hold off;
    xlabel('Thoi gian (s)'); ylabel('Goc tich phan (deg)');
    title('Drift Gy: truoc vs sau hieu chinh  (5 giay)');
    legend('Location','best','FontSize',9);
    grid on;

    % [4] Bang ket qua
    subplot(2,2,4); axis off;
    tbl = {
        'Truc','Offset (deg/s)','STD (deg/s)','STD < 0.05?','Drift Raw 5s','Drift Cal';
        'Gx', sprintf('%+.5f',gx_off), sprintf('%.5f',std_gx), ok(std_gx<0.05), '-', '-';
        'Gy', sprintf('%+.5f',gy_off), sprintf('%.5f',std_gy), ok(std_gy<0.05), ...
              sprintf('%.3f deg',drift_raw(end)), sprintf('%.5f deg',drift_cal(end));
        'Gz', sprintf('%+.5f',gz_off), sprintf('%.5f',std_gz), ok(std_gz<0.05), '-', '-';
    };
    uitable('Data',tbl(2:end,:),'ColumnName',tbl(1,:), ...
        'ColumnWidth',{35,110,90,80,90,90}, ...
        'Units','normalized','Position',[0.53 0.05 0.45 0.30], ...
        'FontSize',9);

    savefig_out(fig1,'bai2a_result.png');

    %% ============================================================
    %  FIGURE 2 - BAI 2B  (2 x 3)
    % ============================================================
    fig2 = figure('Name','Bai 2B - Accel 6-position','NumberTitle','off', ...
                  'Position',[60 60 1200 760]);
    sgtitle('Bai 2B: Hieu chinh Accelerometer (6-position)', ...
            'FontSize',14,'FontWeight','bold');

    x    = 1:n_pos;
    xlbl = pos_labels;

    % [1,1-2] |a| qua 6 vi tri
    subplot(2,3,[1 2]);
    hold on;
    s_off = 0;
    for p = 1:n_pos
        d   = pos_data{p};
        n   = size(d,1);
        idx = s_off + (1:n);
        mag_r = sqrt(d(:,1).^2 + d(:,2).^2 + d(:,3).^2);
        mag_c = sqrt((d(:,1)+ax_off).^2 + (d(:,2)+ay_off).^2 + (d(:,3)+az_off).^2);
        plot(idx, mag_r, 'Color',[0.75 0.75 0.75],'LineWidth',0.7,'HandleVisibility','off');
        plot(idx, mag_c, 'LineWidth',1.5,'HandleVisibility','off');
        text(s_off + n/2, min(mag_c)-0.008, xlbl{p}, ...
            'HorizontalAlignment','center','FontSize',7.5,'FontWeight','bold');
        s_off = s_off + n;
    end
    plot([0 s_off],[1.00 1.00],'k--','LineWidth',1.5,'HandleVisibility','off');
    plot([0 s_off],[1.03 1.03],'b:','LineWidth',1,  'HandleVisibility','off');
    plot([0 s_off],[0.97 0.97],'b:','LineWidth',1,  'HandleVisibility','off');
    hold off;
    xlabel('Mau'); ylabel('|a| (g)');
    title('|a| qua 6 vi tri  (xam=tho | mau=sau calib | -- = 1g | ... = +-3%)');
    ylim([0.93 1.10]); grid on;

    % [1,3] Bar: |a| trung binh truoc/sau
    subplot(2,3,3);
    b = bar(x, [mR, mC],'grouped');
    b(1).FaceColor = [0.6 0.6 0.6]; b(1).DisplayName = 'Truoc calib';
    b(2).FaceColor = [0.2 0.7 0.5]; b(2).DisplayName = 'Sau calib';
    hold on;
    plot([0.5 n_pos+0.5],[1.00 1.00],'k--','LineWidth',1.5,'HandleVisibility','off');
    plot([0.5 n_pos+0.5],[1.03 1.03],'b:','LineWidth',1,  'HandleVisibility','off');
    plot([0.5 n_pos+0.5],[0.97 0.97],'b:','LineWidth',1,  'HandleVisibility','off');
    hold off;
    set(gca,'XTick',x,'XTickLabel',xlbl,'XTickLabelRotation',20,'FontSize',7.5);
    ylabel('|a| (g)');
    title('|a| trung binh truoc/sau calib');
    legend('Location','northeast','FontSize',8);
    ylim([0.93 1.10]); grid on;

    % [2,1] Sai so % sau calib
    subplot(2,3,4);
    bc = bar(x, eC,'FaceColor',[0.2 0.6 0.8]);
    hold on;
    plot([0.5 n_pos+0.5],[ 3  3],'r--','LineWidth',1.5,'HandleVisibility','off');
    plot([0.5 n_pos+0.5],[-3 -3],'r--','LineWidth',1.5,'HandleVisibility','off');
    plot([0.5 n_pos+0.5],[0 0],'k-','LineWidth',1,'HandleVisibility','off');
    hold off;
    set(gca,'XTick',x,'XTickLabel',xlbl,'XTickLabelRotation',20,'FontSize',7.5);
    ylabel('Sai lech (%)');
    title(sprintf('Sai lech %% sau calib  (max=%.3f%%  gioi han +-3%%)',maxErrCal));
    grid on;

    % [2,2] Histogram Ax (Xup vs Xdn)
    subplot(2,3,5);
    d3 = pos_data{3}; d4 = pos_data{4};
    hold on;
    histogram(d3(:,1), 25,'FaceColor',[0.4 0.6 1.0],'EdgeColor','none', ...
        'DisplayName','Ax Xup raw');
    histogram(d4(:,1), 25,'FaceColor',[1.0 0.4 0.4],'EdgeColor','none', ...
        'DisplayName','Ax Xdn raw');
    histogram(d3(:,1)+ax_off, 25,'FaceColor',[0.2 1.0 0.5],'FaceAlpha',0.80, ...
        'EdgeColor','none','DisplayName','Ax Xup cal');
    histogram(d4(:,1)+ax_off, 25,'FaceColor',[1.0 0.8 0.1],'FaceAlpha',0.80, ...
        'EdgeColor','none','DisplayName','Ax Xdn cal');
    plot([ 1  1],[0 60],'k--','LineWidth',1.5,'HandleVisibility','off');
    plot([-1 -1],[0 60],'k--','LineWidth',1.5,'HandleVisibility','off');
    hold off;
    xlabel('Ax (g)'); ylabel('So mau');
    title(sprintf('Phan phoi Ax  ax\\_off = %+.5f g', ax_off));
    legend('Location','best','FontSize',8);
    xlim([-1.2 1.2]); grid on;

    % [2,3] Bang offset day du
    subplot(2,3,6); axis off;
    tbl2 = {
        'Thong so',     'Gia tri',              'Dat chuan?';
        'ax\_off (g)',  sprintf('%+.5f',ax_off), ok(maxErrCal<3);
        'ay\_off (g)',  sprintf('%+.5f',ay_off), ok(maxErrCal<3);
        'az\_off (g)',  sprintf('%+.5f',az_off), ok(maxErrCal<3);
        'gx\_off (d/s)',sprintf('%+.5f',gx_off), ok(std_gx<0.05);
        'gy\_off (d/s)',sprintf('%+.5f',gy_off), ok(std_gy<0.05);
        'gz\_off (d/s)',sprintf('%+.5f',gz_off), ok(std_gz<0.05);
        'Max err calib',sprintf('%.3f%%',maxErrCal), ok(maxErrCal<3);
    };
    uitable('Data',tbl2(2:end,:),'ColumnName',tbl2(1,:), ...
        'ColumnWidth',{90,110,80}, ...
        'Units','normalized','Position',[0.685 0.04 0.295 0.28], ...
        'FontSize',9);

    savefig_out(fig2,'bai2b_result.png');

    %% Luu offset
    fid = fopen('bai2_offsets.txt','w');
    fprintf(fid,'ax_off = %+.6f;  %% g\n',    ax_off);
    fprintf(fid,'ay_off = %+.6f;  %% g\n',    ay_off);
    fprintf(fid,'az_off = %+.6f;  %% g\n',    az_off);
    fprintf(fid,'gx_off = %+.6f;  %% deg/s\n',gx_off);
    fprintf(fid,'gy_off = %+.6f;  %% deg/s\n',gy_off);
    fprintf(fid,'gz_off = %+.6f;  %% deg/s\n',gz_off);
    fclose(fid);
    fprintf('\n[OK] Da luu: bai2a_result.png  bai2b_result.png  bai2_offsets.txt\n');
end

%% ================================================================
%  DOC FILE CSV THUAN TUY (khong co header)
% ================================================================
function data = doc_csv(fname, ncols)
    data = []; fid = fopen(fname,'r');
    if fid == -1
        fprintf('[LOI] Khong mo duoc: %s\n', fname); return;
    end
    rows = {};
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if ~ischar(line) || isempty(line) || line(1)=='%' || line(1)=='#'
            continue;
        end
        v = str2double(strsplit(line,{',',' ','\t'}));
        if length(v) >= ncols && all(~isnan(v(1:ncols)))
            rows{end+1} = v(1:ncols); %#ok<AGROW>
        end
    end
    fclose(fid);
    if ~isempty(rows)
        data = cell2mat(rows');
        fprintf('[OK] %d mau <- %s\n', size(data,1), fname);
    else
        fprintf('[LOI] Khong co du lieu trong: %s\n', fname);
    end
end

%% ================================================================
%  DOC FILE 6-POSITION
%  SUA LOI CHINH: chi lay du lieu SAU khi gap header "Pos N -"
%  Bo qua hoan toan du lieu truoc header dau tien
% ================================================================
function [pos_data, pos_labels, n_pos] = doc_6pos(fname)
    pos_data   = {};
    pos_labels = {};
    n_pos      = 0;
    def_labels = {'Z len','Z xuong','X len','X xuong','Y len','Y xuong'};

    fid = fopen(fname,'r');
    if fid == -1
        fprintf('[LOI] Khong mo duoc: %s\n', fname); return;
    end
    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid); %#ok<AGROW>
    end
    fclose(fid);

    cur_rows    = {};
    cur_label   = '';
    inside_pos  = false;   % chi them data khi da gap header

    for i = 1:length(lines)
        line = strtrim(lines{i});
        if ~ischar(line) || isempty(line), continue; end

        % --- Phat hien dong header "Pos N - ..." ---
        is_header = ~isempty(regexp(line,'^\s*[Pp]os\s+\d','once'));

        if is_header
            % Luu vi tri truoc do (neu co)
            if inside_pos && ~isempty(cur_rows)
                pos_data{end+1}   = cell2mat(cur_rows'); %#ok<AGROW>
                pos_labels{end+1} = cur_label; %#ok<AGROW>
                n_pos  = n_pos + 1;
                cur_rows = {};
            end
            inside_pos = true;

            % Trich ten vi tri tu header
            cur_label = def_labels{min(n_pos+1, 6)};
            tok = regexp(line,'Pos\s+\d+\s*-\s*(.+)','tokens','once');
            if ~isempty(tok)
                lb = strtrim(tok{1});
                lb = strtrim(regexprep(lb,'\(.*\)',''));
                lb = strtrim(regexprep(lb,'→.*',''));
                if length(lb) > 16, lb = lb(1:16); end
                cur_label = lb;
            end
            continue;
        end

        % Bo qua dong huong dan va dong tom tat
        if ~isempty(regexp(line,'Ax\s*=','once')), continue; end
        if length(line)>1 && line(1)=='>', continue; end

        % Chi them data khi da vao mot vi tri (inside_pos = true)
        if inside_pos
            v = str2double(strsplit(line,{',',' ','\t'}));
            if length(v) >= 3 && all(~isnan(v(1:3)))
                cur_rows{end+1} = v(1:3); %#ok<AGROW>
            end
        end
        % Neu chua gap header thi BỎ QUA moi du lieu
    end

    % Luu vi tri cuoi
    if inside_pos && ~isempty(cur_rows)
        pos_data{end+1}   = cell2mat(cur_rows');
        pos_labels{end+1} = cur_label;
        n_pos = n_pos + 1;
    end

    fprintf('[OK] %d vi tri <- %s\n', n_pos, fname);
    for p = 1:n_pos
        fprintf('  Pos %d: %-18s  %d mau\n', p, pos_labels{p}, size(pos_data{p},1));
    end
end

%% ================================================================
%  HAM TIEN ICH
% ================================================================
function savefig_out(fig, fname)
    try
        exportgraphics(fig, fname, 'Resolution', 150);
    catch
        saveas(fig, fname);
    end
    fprintf('[OK] Luu: %s\n', fname);
end

function s = ok(cond)
    if cond, s = 'OK'; else, s = '!! CANH BAO'; end
end
%% ================================================================
%  BAI 3: ANH HUONG GIA TOC TRONG TRUONG
%  File: bai3_data.csv (7 cot: angle_deg, ax, ay, az, gx, gy, gz)
%  moi goc: 200 mau, thu tu: 0, 15, 30, 45, 60, 90 do
% ================================================================
function task3_gravity()
    fprintf('\n========================================\n');
    fprintf('  BAI 3: ANH HUONG GIA TOC TRONG TRUONG\n');
    fprintf('========================================\n');
    fprintf('File can: bai3_data.csv\n');
    fprintf('Dinh dang: angle_deg, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps\n');
    fprintf('(200 mau cho moi goc: 0,15,30,45,60,90 do)\n\n');

    fname = input('Nhap ten file (Enter=bai3_data.csv): ','s');
    if isempty(fname), fname = 'bai3_data.csv'; end

    data = load_csv(fname, 4);
    if isempty(data), return; end

    % Lay cot angle (cot 1) va ax,ay,az (cot 2,3,4)
    angles_data = data(:,1);
    ax_all = data(:,2);
    ay_all = data(:,3);
    az_all = data(:,4);

    % Cac goc ly thuyet
    theo_angles = [0; 15; 30; 45; 60; 90];
    theo_ax = sind(theo_angles);
    theo_ay = zeros(6,1);
    theo_az = cosd(theo_angles);
    theo_mag = ones(6,1);

    % Tinh mean moi goc
    unique_angles = unique(angles_data);
    n_angles = min(length(unique_angles), 6);
    meas_ax  = zeros(n_angles,1);
    meas_ay  = zeros(n_angles,1);
    meas_az  = zeros(n_angles,1);
    meas_mag = zeros(n_angles,1);
    actual_angles = zeros(n_angles,1);

    for i = 1:n_angles
        mask = abs(angles_data - unique_angles(i)) < 1;
        meas_ax(i)  = mean(ax_all(mask));
        meas_ay(i)  = mean(ay_all(mask));
        meas_az(i)  = mean(az_all(mask));
        meas_mag(i) = mean(sqrt(ax_all(mask).^2+ay_all(mask).^2+az_all(mask).^2));
        actual_angles(i) = unique_angles(i);
    end

    % Sai so so voi ly thuyet (dung goc goc nhat)
    err_ax  = meas_ax(1:min(n_angles,6))  - theo_ax(1:min(n_angles,6));
    err_az  = meas_az(1:min(n_angles,6))  - theo_az(1:min(n_angles,6));
    err_mag = meas_mag(1:min(n_angles,6)) - theo_mag(1:min(n_angles,6));

    fprintf('\n--- KET QUA DO THUC TE VS LY THUYET ---\n');
    fprintf('Goc | Ax_do  | Ax_LT  | Ay_do  | Az_do  | Az_LT  | |a|    | Sl%%\n');
    fprintf('----|--------|--------|--------|--------|--------|--------|-----\n');
    for i=1:n_angles
        j = min(i,6);
        fprintf('%3.0f | %+.4f | %+.4f | %+.4f | %+.4f | %+.4f | %.4f | %.1f%%\n',...
            actual_angles(i), meas_ax(i), theo_ax(j), meas_ay(i),...
            meas_az(i), theo_az(j), meas_mag(i), abs(meas_mag(i)-1)*100);
    end

    % --- Ve do thi ---
    fig = figure('Name','Bai 3 - Trong truong','NumberTitle','off','Position',[100 100 1400 800]);
    sgtitle('Bai 3: Phan bo vector g theo goc nghieng','FontSize',14,'FontWeight','bold');

    ang_plot = actual_angles(1:min(n_angles,6));

    % Ax do vs ly thuyet
    subplot(2,3,1);
    plot(theo_angles, theo_ax,   'b--o','LineWidth',2,'MarkerSize',8,'DisplayName','Ly thuyet'); hold on;
    plot(ang_plot,    meas_ax(1:min(n_angles,6)), 'r-s','LineWidth',2,'MarkerSize',8,'DisplayName','Do thuc te');
    xlabel('Goc nghieng (do)'); ylabel('Ax (g)');
    title('Ax: Do vs Ly thuyet  [= sin(θ)]');
    legend; grid on; hold off;

    % Az do vs ly thuyet
    subplot(2,3,2);
    plot(theo_angles, theo_az,   'b--o','LineWidth',2,'MarkerSize',8,'DisplayName','Ly thuyet'); hold on;
    plot(ang_plot,    meas_az(1:min(n_angles,6)), 'r-s','LineWidth',2,'MarkerSize',8,'DisplayName','Do thuc te');
    xlabel('Goc nghieng (do)'); ylabel('Az (g)');
    title('Az: Do vs Ly thuyet  [= cos(θ)]');
    legend; grid on; hold off;

    % |a| theo goc - phai ~ 1 g
    subplot(2,3,3);
    plot(theo_angles, theo_mag,'b--','LineWidth',2,'DisplayName','Ly tuong 1g'); hold on;
    plot(ang_plot,    meas_mag(1:min(n_angles,6)),'r-o','LineWidth',2,'MarkerSize',8,'DisplayName','|a| do');
    yline(1.03,'k:'); yline(0.97,'k:','Label','±3%');
    xlabel('Goc nghieng (do)'); ylabel('|a| (g)');
    title('Bien do tong |a| theo goc (phai ~ 1g)');
    legend; ylim([0.85 1.15]); grid on; hold off;

    % Sai so Ax
    subplot(2,3,4);
    bar(ang_plot, err_ax*100,'FaceColor','#e63946');
    yline(0,'k--'); yline(3,'b:'); yline(-3,'b:','Label','±3%');
    xlabel('Goc (do)'); ylabel('Sai so (%)');
    title('Sai so Ax (do - ly thuyet)'); grid on;

    % Sai so Az
    subplot(2,3,5);
    bar(ang_plot, err_az*100,'FaceColor','#457b9d');
    yline(0,'k--'); yline(3,'b:'); yline(-3,'b:','Label','±3%');
    xlabel('Goc (do)'); ylabel('Sai so (%)');
    title('Sai so Az (do - ly thuyet)'); grid on;

    % Ax^2 + Ay^2 + Az^2 = 1 kiem tra
    subplot(2,3,6);
    sum_sq = meas_ax.^2 + meas_ay.^2 + meas_az.^2;
    bar(ang_plot, (sum_sq(1:n_angles)-1)*100,'FaceColor','#2a9d8f');
    yline(0,'k--'); yline(3,'r:'); yline(-3,'r:','Label','Gioi han 3%');
    xlabel('Goc (do)'); ylabel('Sai lech Ax²+Ay²+Az² (%)');
    title('Kiem tra: Ax²+Ay²+Az² = 1'); grid on;

    save_figure(fig,'bai3_result.png');
    fprintf('\n[OK] Bai 3 hoan tat. Da luu: bai3_result.png\n\n');
end

%% ================================================================
%  BAI 4: BO LOC TIN HIEU - COMPLEMENTARY & KALMAN
%  File: bai4_data.csv
%  Cot: time_ms, roll_accel, pitch_accel, roll_cf, pitch_cf, roll_kf, pitch_kf
% ================================================================
function task4_filter()
    fprintf('\n========================================\n');
    fprintf('  BAI 4: BO LOC TIN HIEU\n');
    fprintf('========================================\n');
    fprintf('File can: bai4_data.csv\n');
    fprintf('Cot: time_ms, roll_accel, pitch_accel, roll_cf, pitch_cf, roll_kf, pitch_kf\n\n');

    fname = input('Nhap ten file (Enter=bai4_data.csv): ','s');
    if isempty(fname), fname = 'bai4_data.csv'; end

    data = load_csv(fname, 7);
    if isempty(data), return; end

    t       = data(:,1) / 1000;   % ms -> s
    roll_a  = data(:,2);
    pitch_a = data(:,3);
    roll_cf = data(:,4);
    pitch_cf= data(:,5);
    roll_kf = data(:,6);
    pitch_kf= data(:,7);

    % Tinh chi so danh gia
    % Sai lech RMS giua cac phuong phap
    rms_roll_cf_vs_kf  = rms(roll_cf  - roll_kf);
    rms_pitch_cf_vs_kf = rms(pitch_cf - pitch_kf);
    rms_roll_a_vs_kf   = rms(roll_a   - roll_kf);
    rms_pitch_a_vs_kf  = rms(pitch_a  - pitch_kf);

    % Drift gyro: do bien dong cuoi/dau khi dung yen
    roll_drift  = roll_kf(end)  - roll_kf(1);
    pitch_drift = pitch_kf(end) - pitch_kf(1);

    fprintf('\n--- CHI SO DANH GIA ---\n');
    fprintf('RMS(roll_CF  - roll_KF)  = %.4f deg\n', rms_roll_cf_vs_kf);
    fprintf('RMS(pitch_CF - pitch_KF) = %.4f deg\n', rms_pitch_cf_vs_kf);
    fprintf('RMS(roll_Acc - roll_KF)  = %.4f deg  (nhieu Acc)\n', rms_roll_a_vs_kf);
    fprintf('RMS(pitch_Acc- pitch_KF) = %.4f deg  (nhieu Acc)\n', rms_pitch_a_vs_kf);
    fprintf('Drift Roll  (KF): %.4f deg trong %.1f s\n', roll_drift,  t(end)-t(1));
    fprintf('Drift Pitch (KF): %.4f deg trong %.1f s\n', pitch_drift, t(end)-t(1));

    % --- Ve do thi ---
    fig = figure('Name','Bai 4 - Bo loc','NumberTitle','off','Position',[100 100 1400 900]);
    sgtitle('Bai 4: So sanh Raw / Complementary / Kalman Filter','FontSize',14,'FontWeight','bold');

    % Roll - 3 phuong phap
    subplot(3,2,1);
    plot(t, roll_a,  'Color',[0.7 0.7 0.7],'LineWidth',0.8,'DisplayName','Accel (raw)'); hold on;
    plot(t, roll_cf, '#e63946','LineWidth',1.5,'DisplayName',sprintf('Complementary (CF)'));
    plot(t, roll_kf, '#1d3557','LineWidth',2,'DisplayName','Kalman (KF)');
    xlabel('Thoi gian (s)'); ylabel('Roll (deg)');
    title('Roll: Accel vs CF vs Kalman'); legend('Location','best'); grid on; hold off;

    % Pitch - 3 phuong phap
    subplot(3,2,2);
    plot(t, pitch_a,  'Color',[0.7 0.7 0.7],'LineWidth',0.8,'DisplayName','Accel (raw)'); hold on;
    plot(t, pitch_cf, '#2a9d8f','LineWidth',1.5,'DisplayName','Complementary (CF)');
    plot(t, pitch_kf, '#e76f51','LineWidth',2,'DisplayName','Kalman (KF)');
    xlabel('Thoi gian (s)'); ylabel('Pitch (deg)');
    title('Pitch: Accel vs CF vs Kalman'); legend('Location','best'); grid on; hold off;

    % Sai lech CF vs KF (Roll)
    subplot(3,2,3);
    plot(t, roll_cf - roll_kf, '#e63946','LineWidth',1.2);
    yline(0,'k--'); yline(1,'b:'); yline(-1,'b:','Label','±1 deg');
    xlabel('Thoi gian (s)'); ylabel('Sai lech (deg)');
    title(sprintf('Roll: CF - KF  (RMS=%.4f deg)',rms_roll_cf_vs_kf)); grid on;

    % Sai lech CF vs KF (Pitch)
    subplot(3,2,4);
    plot(t, pitch_cf - pitch_kf, '#2a9d8f','LineWidth',1.2);
    yline(0,'k--'); yline(1,'b:'); yline(-1,'b:','Label','±1 deg');
    xlabel('Thoi gian (s)'); ylabel('Sai lech (deg)');
    title(sprintf('Pitch: CF - KF  (RMS=%.4f deg)',rms_pitch_cf_vs_kf)); grid on;

    % Pho tan so cua tin hieu Roll (FFT de thay noise)
    subplot(3,2,5);
    Fs = 1/mean(diff(t));
    N  = length(roll_a);
    f  = (0:floor(N/2)) * Fs/N;
    Y_a  = abs(fft(roll_a  - mean(roll_a)));
    Y_cf = abs(fft(roll_cf - mean(roll_cf)));
    Y_kf = abs(fft(roll_kf - mean(roll_kf)));
    semilogy(f, Y_a(1:floor(N/2)+1),'Color',[0.7 0.7 0.7],'DisplayName','Accel'); hold on;
    semilogy(f, Y_cf(1:floor(N/2)+1),'r','DisplayName','CF');
    semilogy(f, Y_kf(1:floor(N/2)+1),'b','LineWidth',1.5,'DisplayName','KF');
    xlabel('Tan so (Hz)'); ylabel('Bien do (log)');
    title('Pho tan so Roll (FFT) - danh gia loc nhieu'); legend; grid on;
    xlim([0 min(Fs/2, 50)]); hold off;

    % Bang so sanh
    subplot(3,2,6);
    axis off;
    tbl = {
        'Tieu chi','Accel','CF','Kalman';
        'Chong drift Gyro','Khong','Tot','Rat tot';
        'Chong nhieu Accel','Khong','Tot','Rat tot';
        'Do phuc tap','Khong','Thap','Trung binh';
        sprintf('RMS vs KF (Roll)',''),'-', ...
            sprintf('%.4f',rms_roll_cf_vs_kf), '0 (ref)';
        sprintf('RMS vs KF (Pitch)',''), '-', ...
            sprintf('%.4f',rms_pitch_cf_vs_kf),'0 (ref)';
        'Drift Roll (deg)','-', sprintf('%.3f',rms(roll_cf)),sprintf('%.3f',roll_drift);
    };
    uitable(fig,'Data',tbl(2:end,:),'ColumnName',tbl(1,:),...
        'ColumnWidth',{130,60,70,70},'Units','normalized',...
        'Position',[0.52 0.02 0.46 0.26],'FontSize',8);

    save_figure(fig,'bai4_result.png');
    fprintf('\n[OK] Bai 4 hoan tat. Da luu: bai4_result.png\n\n');
end

%% ================================================================
%  BAI 5: DO SHOCK & PHAN TICH FFT
%  File A: bai5_shock.csv  (6 cot: time_ms,ax,ay,az,mag,shock_flag)
%  File B: bai5_vib.txt    (1 cot: |a|-1g, 512 mau @ 500Hz)
% ================================================================
function task5_shock_fft()
    fprintf('\n========================================\n');
    fprintf('  BAI 5: DO SHOCK & PHAN TICH FFT\n');
    fprintf('========================================\n');
    fprintf('Chon che do:\n');
    fprintf('  S - Phan tich Shock (file: bai5_shock.csv)\n');
    fprintf('  F - Phan tich FFT dao dong (file: bai5_vib.txt)\n');
    fprintf('  B - Ca hai\n\n');
    mode = upper(strtrim(input('Chon (S/F/B): ','s')));
    if isempty(mode), mode = 'B'; end

    if mode=='S' || mode=='B'
        task5_analyze_shock();
    end
    if mode=='F' || mode=='B'
        task5_analyze_fft();
    end
end

function task5_analyze_shock()
    fprintf('\n[5A] PHAN TICH SHOCK\n');
    fname = input('File shock (Enter=bai5_shock.csv): ','s');
    if isempty(fname), fname = 'bai5_shock.csv'; end
    data = load_csv(fname, 5);
    if isempty(data), return; end

    t    = data(:,1) / 1000;  % ms -> s
    ax   = data(:,2);
    ay   = data(:,3);
    az   = data(:,4);
    mag  = data(:,5);

    % neu co cot thu 6 (shock flag)
    if size(data,2) >= 6
        shock_flag = data(:,6);
    else
        shock_flag = double(abs(mag-1.0) > 2.0);
    end

    % Tinh biet so
    mag_ac = mag - 1.0;  % loai bo DC (1g)
    [peak_val, peak_idx] = max(abs(mag_ac));
    settle_thresh = peak_val * 0.05;
    settle_idx = length(mag_ac);
    for i = length(mag_ac):-1:1
        if abs(mag_ac(i)) > settle_thresh
            settle_idx = i; break;
        end
    end

    % Tinh tan so dao dong tu nhien
    [pks, locs] = findpeaks(abs(mag_ac),'MinPeakHeight',settle_thresh,...
                             'MinPeakDistance',round(0.05*(t(end)-t(1))*length(t)/(t(end)-t(1))));
    if length(locs) >= 2
        nat_freq = (length(locs)-1) / (t(locs(end)) - t(locs(1)));
    else
        nat_freq = 0;
    end

    fprintf('\n--- KET QUA PHAN TICH SHOCK ---\n');
    fprintf('  Bien do dinh (Peak):    %.4f g\n', peak_val);
    fprintf('  Thoi gian dinh:         %.3f s\n', t(peak_idx));
    fprintf('  Settling time (5%%):     %.3f s\n', t(settle_idx));
    fprintf('  Tan so dao dong tu nhien: %.2f Hz\n', nat_freq);
    fprintf('  RMS gia toc:            %.4f g\n', rms(mag_ac));

    fig = figure('Name','Bai 5A - Shock','NumberTitle','off','Position',[100 100 1400 700]);
    sgtitle('Bai 5A: Phan tich Shock va Xung gia toc','FontSize',14,'FontWeight','bold');

    % 3 truc gia toc
    subplot(2,3,1);
    plot(t,ax,'r',t,ay,'g',t,az,'b','LineWidth',1);
    xlabel('Thoi gian (s)'); ylabel('Gia toc (g)');
    title('Ax, Ay, Az theo thoi gian'); legend('Ax','Ay','Az'); grid on;

    % Mag va shock flag
    subplot(2,3,2);
    yyaxis left;
    plot(t, mag_ac,'k','LineWidth',1.5,'DisplayName','|a|-1g'); hold on;
    yline(0,'k--');
    if length(locs)>0
        plot(t(locs), mag_ac(locs),'rv','MarkerSize',10,'DisplayName','Dinh dao dong');
    end
    plot(t(peak_idx), mag_ac(peak_idx),'r*','MarkerSize',14,'DisplayName',...
        sprintf('Peak=%.4fg',peak_val));
    xline(t(settle_idx),'b--','Label',sprintf('Settle=%.3fs',t(settle_idx)),...
        'LineWidth',1.5);
    ylabel('Gia toc (g)');
    yyaxis right;
    area(t, shock_flag*max(abs(mag_ac))*0.3,'FaceColor','y','FaceAlpha',0.3,'DisplayName','Shock zone');
    ylabel('Shock');
    xlabel('Thoi gian (s)'); title('Bien do shock & Vung shock'); legend('Location','best'); grid on;

    % Chi tiet quanh shock
    if peak_idx > 0
        win  = round(0.2 * length(t));
        i1   = max(1, peak_idx - win);
        i2   = min(length(t), peak_idx + win);
        subplot(2,3,3);
        plot(t(i1:i2), mag_ac(i1:i2),'k','LineWidth',1.5); hold on;
        yline(settle_thresh,'r--','Label',sprintf('5%%peak=%.4f',settle_thresh));
        yline(-settle_thresh,'r--');
        plot(t(peak_idx), mag_ac(peak_idx),'r*','MarkerSize',14);
        xlabel('Thoi gian (s)'); ylabel('|a|-1g (g)');
        title('Zoom quanh dinh shock'); grid on; hold off;
    end

    % FFT cua mag_ac de tim tan so shock
    subplot(2,3,4);
    Fs = 1/mean(diff(t));
    N  = length(mag_ac);
    f  = (0:floor(N/2)) * Fs/N;
    Y  = abs(fft(mag_ac));
    stem(f, Y(1:floor(N/2)+1),'filled','MarkerSize',3,'Color','#457b9d');
    [~,dom_idx] = max(Y(2:floor(N/2)+1));
    dom_idx = dom_idx + 1;
    xline(f(dom_idx),'r--','Label',sprintf('f=%.2fHz',f(dom_idx)),'LineWidth',1.5);
    xlabel('Tan so (Hz)'); ylabel('Bien do');
    title('FFT cua tin hieu shock'); grid on; xlim([0 min(Fs/2,100)]);

    % So sanh 3 truc
    subplot(2,3,5);
    bar([rms(ax-mean(ax)), rms(ay-mean(ay)), rms(az-mean(az))],...
        'FaceColor','#e63946');
    set(gca,'XTickLabel',{'RMS Ax','RMS Ay','RMS Az'});
    ylabel('RMS (g)'); title('RMS tung truc (loai bo mean)'); grid on;

    % Bang ket qua
    subplot(2,3,6);
    axis off;
    tbl = {
        'Bien do dinh (g)',         sprintf('%.4f', peak_val);
        'Thoi gian dinh (s)',       sprintf('%.3f', t(peak_idx));
        'Settling time (s)',        sprintf('%.3f', t(settle_idx));
        'Tan so TN (Hz)',           sprintf('%.2f', nat_freq);
        'RMS |a|-1g (g)',           sprintf('%.4f', rms(mag_ac));
        'So dinh dao dong',         sprintf('%d',   length(locs));
        'Fs (Hz)',                  sprintf('%.1f', Fs);
        'Thoi luong (s)',           sprintf('%.3f', t(end)-t(1));
    };
    uitable(fig,'Data',tbl,'ColumnName',{'Thong so','Gia tri'},...
        'ColumnWidth',{150,90},'Units','normalized',...
        'Position',[0.68 0.02 0.30 0.25],'FontSize',8);

    save_figure(fig,'bai5a_shock_result.png');
    fprintf('[OK] Da luu: bai5a_shock_result.png\n');
end

function task5_analyze_fft()
    fprintf('\n[5B] PHAN TICH FFT DAO DONG\n');
    fname = input('File vibration (Enter=bai5_vib.txt): ','s');
    if isempty(fname), fname = 'bai5_vib.txt'; end

    % Doc file vib - moi dong 1 so
    sig = [];
    fid = fopen(fname,'r');
    if fid == -1
        fprintf('[LOI] Khong mo duoc file: %s\n', fname);
        return;
    end
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || line(1)=='#' || ...
           startsWith(upper(line),'SAMPLE') || startsWith(line,'fs=') || startsWith(line,'n=')
            continue;
        end
        val = str2double(line);
        if ~isnan(val), sig(end+1,1) = val; end %#ok<AGROW>
    end
    fclose(fid);

    if length(sig) < 16
        fprintf('[LOI] Khong du du lieu (can it nhat 16 mau, doc duoc %d).\n',length(sig));
        return;
    end

    Fs      = 500;   % Hz - khop voi Arduino SMPLRT_DIV=1
    N       = length(sig);
    t_vib   = (0:N-1)/Fs;
    LPF_CUT = 50;    % Hz

    % Low-pass filter
    if exist('butter','file')
        [b,a]    = butter(4, LPF_CUT/(Fs/2), 'low');
        sig_filt = filtfilt(b, a, sig);
    else
        sig_filt = sig;
        fprintf('[WARN] Khong co Signal Processing Toolbox, bo qua LPF.\n');
    end

    % FFT (Hanning window)
    win      = hanning(N);
    Y_raw    = abs(fft(sig   .* win)) * 2/N;
    Y_filt   = abs(fft(sig_filt .* win)) * 2/N;
    f_axis   = (0:floor(N/2)) * Fs/N;
    Y_raw_h  = Y_raw(1:floor(N/2)+1);
    Y_filt_h = Y_filt(1:floor(N/2)+1);
    Y_db     = 20*log10(Y_raw_h + 1e-10);

    % Tan so chu dao
    [~,dom_idx]  = max(Y_raw_h(2:end));
    dom_f_raw    = f_axis(dom_idx+1);
    [~,dom_fidx] = max(Y_filt_h(2:end));
    dom_f_filt   = f_axis(dom_fidx+1);

    % Top 5 tan so
    [~,top_idx]  = sort(Y_raw_h,'descend');
    top_idx      = top_idx(top_idx > 1);  % bo DC

    % Phan tich shock
    peak_val     = max(abs(sig));
    rms_sig      = rms(sig);
    settle_t     = t_vib(end);
    settle_thresh= peak_val*0.05;
    for i = N:-1:1
        if abs(sig(i)) > settle_thresh, settle_t = t_vib(i); break; end
    end

    % Hieu qua LPF
    e_raw  = sum(Y_raw_h.^2);
    e_filt = sum(Y_filt_h.^2);
    kept   = e_filt/e_raw*100;

    fprintf('\n--- KET QUA FFT ---\n');
    fprintf('  So mau: %d  |  Fs: %d Hz  |  Do phan giai: %.3f Hz/bin\n',N,Fs,Fs/N);
    fprintf('  Bien do dinh:    %.5f g\n', peak_val);
    fprintf('  RMS:             %.5f g\n', rms_sig);
    fprintf('  Settling time:   %.3f s\n', settle_t);
    fprintf('  Tan so chu dao (Raw):  %.2f Hz\n', dom_f_raw);
    fprintf('  Tan so chu dao (LPF):  %.2f Hz\n', dom_f_filt);
    fprintf('  LPF giu lai: %.1f%% nang luong\n', kept);
    fprintf('\n  Top 5 tan so noi bat:\n');
    for i = 1:min(5, length(top_idx))
        fprintf('    %d. %.2f Hz  |  %.5f g\n', i, f_axis(top_idx(i)), Y_raw_h(top_idx(i)));
    end

    fig = figure('Name','Bai 5B - FFT','NumberTitle','off','Position',[100 100 1400 900]);
    sgtitle('Bai 5B: Phan tich tan so dao dong (FFT)','FontSize',14,'FontWeight','bold');

    % Tin hieu thoi gian
    subplot(3,2,[1 2]);
    plot(t_vib, sig,      'Color',[0.7 0.7 0.7],'LineWidth',0.8,'DisplayName','Tin hieu tho'); hold on;
    plot(t_vib, sig_filt, 'r','LineWidth',1.8,'DisplayName',sprintf('Sau LPF %dHz',LPF_CUT));
    yline(0,'k--');
    xlabel('Thoi gian (s)'); ylabel('Gia toc (g)');
    title('Tin hieu dao dong |a|-1g theo thoi gian'); legend; grid on; hold off;

    % Pho bien do - Raw
    subplot(3,2,3);
    plot(f_axis, Y_raw_h,'Color','#457b9d','LineWidth',1.2);
    xline(dom_f_raw,'r--','LineWidth',1.5,'Label',sprintf('%.2f Hz',dom_f_raw));
    plot(f_axis(top_idx(1)), Y_raw_h(top_idx(1)),'r*','MarkerSize',12);
    xlabel('Tan so (Hz)'); ylabel('Bien do (g)');
    title('Pho FFT - Tin hieu tho'); grid on; xlim([0 Fs/2]);

    % Pho bien do - Sau LPF
    subplot(3,2,4);
    plot(f_axis, Y_filt_h,'Color','#2a9d8f','LineWidth',1.2);
    xline(dom_f_filt,'r--','LineWidth',1.5,'Label',sprintf('%.2f Hz',dom_f_filt));
    xline(LPF_CUT,'b:','LineWidth',1.5,'Label',sprintf('LPF %dHz',LPF_CUT));
    xlabel('Tan so (Hz)'); ylabel('Bien do (g)');
    title(sprintf('Pho FFT - Sau LPF %d Hz',LPF_CUT)); grid on; xlim([0 Fs/2]);

    % Pho cong suat dB
    subplot(3,2,5);
    plot(f_axis, Y_db,'Color','#6a0572','LineWidth',1.0);
    yline(-3,'Color','orange','LineStyle','--','Label','-3dB');
    yline(-20,'Color',[0.5 0.5 0.5],'LineStyle','--','Label','-20dB');
    xline(LPF_CUT,'b:','LineWidth',1.5,'Label',sprintf('LPF %dHz',LPF_CUT));
    xlabel('Tan so (Hz)'); ylabel('Cong suat (dB)');
    title('Pho cong suat (dB scale)'); grid on;
    xlim([0 Fs/2]); ylim([-80 max(Y_db)+5]);

    % Bang ket qua
    subplot(3,2,6);
    axis off;
    tbl = {
        'So mau N',              num2str(N);
        'Fs (Hz)',               num2str(Fs);
        'Do phan giai (Hz/bin)', sprintf('%.3f',Fs/N);
        'Bien do dinh (g)',      sprintf('%.5f',peak_val);
        'RMS (g)',               sprintf('%.5f',rms_sig);
        'Settling time (s)',     sprintf('%.3f',settle_t);
        'f chu dao - Raw (Hz)',  sprintf('%.2f',dom_f_raw);
        'f chu dao - LPF (Hz)',  sprintf('%.2f',dom_f_filt);
        'LPF cutoff (Hz)',       num2str(LPF_CUT);
        'LPF giu lai (%)',       sprintf('%.1f',kept);
    };
    uitable(fig,'Data',tbl,'ColumnName',{'Thong so','Gia tri'},...
        'ColumnWidth',{160,100},'Units','normalized',...
        'Position',[0.52 0.02 0.46 0.30],'FontSize',8);

    save_figure(fig,'bai5b_fft_result.png');
    fprintf('[OK] Da luu: bai5b_fft_result.png\n\n');
end

%% ================================================================
%  BAI 6: UOC LUONG GOC ROLL / PITCH / YAW
%  File: bai6_data.csv
%  Cot: time_ms,roll_accel,pitch_accel,roll_gyro,pitch_gyro,yaw_gyro,
%       roll_kf,pitch_kf,yaw_kf
% ================================================================
function task6_angles()
    fprintf('\n========================================\n');
    fprintf('  BAI 6: UOC LUONG GOC ROLL/PITCH/YAW\n');
    fprintf('========================================\n');
    fprintf('File can: bai6_data.csv\n');
    fprintf('Cot: time_ms,roll_accel,pitch_accel,roll_gyro,pitch_gyro,yaw_gyro,roll_kf,pitch_kf,yaw_kf\n\n');

    fname = input('Nhap ten file (Enter=bai6_data.csv): ','s');
    if isempty(fname), fname = 'bai6_data.csv'; end

    data = load_csv(fname, 9);
    if isempty(data), return; end

    t        = data(:,1)/1000;
    roll_a   = data(:,2); pitch_a   = data(:,3);
    roll_g   = data(:,4); pitch_g   = data(:,5); yaw_g = data(:,6);
    roll_kf  = data(:,7); pitch_kf  = data(:,8); yaw_kf= data(:,9);

    % Phan tich drift
    Fs          = 1/mean(diff(t));
    drift_rate_r = (roll_kf(end)-roll_kf(1))/(t(end)-t(1));
    drift_rate_p = (pitch_kf(end)-pitch_kf(1))/(t(end)-t(1));
    drift_yaw    = yaw_kf(end)-yaw_kf(1);

    % Sai so RMS
    rms_roll  = rms(roll_a  - roll_kf);
    rms_pitch = rms(pitch_a - pitch_kf);

    fprintf('\n--- DANH GIA ---\n');
    fprintf('  Thoi gian ghi: %.1f s  |  Fs: %.1f Hz\n', t(end)-t(1), Fs);
    fprintf('  RMS(Accel vs KF) Roll:  %.4f deg\n', rms_roll);
    fprintf('  RMS(Accel vs KF) Pitch: %.4f deg\n', rms_pitch);
    fprintf('  Drift Roll  (KF): %.4f deg/s\n', drift_rate_r);
    fprintf('  Drift Pitch (KF): %.4f deg/s\n', drift_rate_p);
    fprintf('  Drift Yaw (Gyro tich phan): %.2f deg trong %.1f s\n', drift_yaw, t(end)-t(1));

    fig = figure('Name','Bai 6 - Goc','NumberTitle','off','Position',[100 100 1400 900]);
    sgtitle('Bai 6: Uoc luong goc Roll / Pitch / Yaw','FontSize',14,'FontWeight','bold');

    % Roll - 3 phuong phap
    subplot(3,3,1);
    plot(t,roll_a, 'Color',[0.7 0.7 0.7],'LineWidth',0.8,'DisplayName','Accel'); hold on;
    plot(t,roll_g, '#f4a261','LineWidth',1.2,'DisplayName','Gyro integral');
    plot(t,roll_kf,'#1d3557','LineWidth',2,'DisplayName','Kalman');
    xlabel('t (s)'); ylabel('Roll (deg)'); title('ROLL'); legend('Location','best'); grid on; hold off;

    % Pitch
    subplot(3,3,2);
    plot(t,pitch_a, 'Color',[0.7 0.7 0.7],'LineWidth',0.8,'DisplayName','Accel'); hold on;
    plot(t,pitch_g, '#f4a261','LineWidth',1.2,'DisplayName','Gyro integral');
    plot(t,pitch_kf,'#e63946','LineWidth',2,'DisplayName','Kalman');
    xlabel('t (s)'); ylabel('Pitch (deg)'); title('PITCH'); legend('Location','best'); grid on; hold off;

    % Yaw
    subplot(3,3,3);
    plot(t,yaw_g, '#2a9d8f','LineWidth',1.5,'DisplayName','Gyro integral'); hold on;
    plot(t,yaw_kf,'#6a0572','LineWidth',2,'DisplayName','Kalman Yaw (drift)');
    xlabel('t (s)'); ylabel('Yaw (deg)');
    title('YAW (drift - can Magnetometer de chinh xac)');
    legend; grid on; hold off;

    % Sai lech Roll: Gyro vs KF (drift gyro)
    subplot(3,3,4);
    plot(t, roll_g - roll_kf, '#f4a261','LineWidth',1.2);
    yline(0,'k--'); xlabel('t (s)'); ylabel('Sai lech (deg)');
    title(sprintf('Roll: Gyro drift vs KF  (drift=%.4f deg/s)',drift_rate_r)); grid on;

    % Sai lech Pitch
    subplot(3,3,5);
    plot(t, pitch_g - pitch_kf, '#e63946','LineWidth',1.2);
    yline(0,'k--'); xlabel('t (s)'); ylabel('Sai lech (deg)');
    title(sprintf('Pitch: Gyro drift vs KF  (drift=%.4f deg/s)',drift_rate_p)); grid on;

    % Sai lech Accel vs KF (nhieu Accel)
    subplot(3,3,6);
    plot(t, roll_a-roll_kf,'b','LineWidth',0.8,'DisplayName','Roll'); hold on;
    plot(t, pitch_a-pitch_kf,'r','LineWidth',0.8,'DisplayName','Pitch');
    yline(0,'k--'); xlabel('t (s)'); ylabel('Sai lech (deg)');
    title('Nhieu Accel vs Kalman'); legend; grid on; hold off;

    % Hoa rose (phan phoi goc)
    subplot(3,3,7);
    polarhistogram(deg2rad(roll_kf), 36,'FaceColor','#457b9d');
    title('Phan bo Roll (polar)');

    subplot(3,3,8);
    polarhistogram(deg2rad(pitch_kf), 36,'FaceColor','#e63946');
    title('Phan bo Pitch (polar)');

    % Bang
    subplot(3,3,9);
    axis off;
    tbl = {
        'Thoi luong (s)',      sprintf('%.1f',t(end)-t(1));
        'Fs (Hz)',             sprintf('%.1f',Fs);
        'RMS Roll Acc-KF',     sprintf('%.4f deg',rms_roll);
        'RMS Pitch Acc-KF',    sprintf('%.4f deg',rms_pitch);
        'Drift Roll (deg/s)',  sprintf('%.4f',drift_rate_r);
        'Drift Pitch (deg/s)', sprintf('%.4f',drift_rate_p);
        'Drift Yaw (deg)',     sprintf('%.2f',drift_yaw);
        'Roll max (KF)',       sprintf('%.2f deg',max(abs(roll_kf)));
        'Pitch max (KF)',      sprintf('%.2f deg',max(abs(pitch_kf)));
    };
    uitable(fig,'Data',tbl,'ColumnName',{'Thong so','Gia tri'},...
        'ColumnWidth',{145,100},'Units','normalized',...
        'Position',[0.68 0.02 0.30 0.28],'FontSize',8);

    save_figure(fig,'bai6_result.png');
    fprintf('\n[OK] Bai 6 hoan tat. Da luu: bai6_result.png\n\n');
end

%% ================================================================
%  BAI 7: BAI TAP TONG HOP - DASHBOARD DAY DU
%  File: bai7_data.csv
%  Cot: time_ms,roll_kf,pitch_kf,yaw_gyro,temp_c,accel_mag,shock_flag
% ================================================================
function task7_summary()
    fprintf('\n========================================\n');
    fprintf('  BAI 7: BAI TAP TONG HOP - DASHBOARD\n');
    fprintf('========================================\n');
    fprintf('File can: bai7_data.csv\n');
    fprintf('Cot: time_ms, roll_kf, pitch_kf, yaw_gyro, temp_c, accel_mag, shock_flag\n\n');

    fname = input('Nhap ten file (Enter=bai7_data.csv): ','s');
    if isempty(fname), fname = 'bai7_data.csv'; end

    data = load_csv(fname, 6);
    if isempty(data), return; end

    t       = data(:,1)/1000;
    roll    = data(:,2);
    pitch   = data(:,3);
    yaw     = data(:,4);
    temp    = data(:,5);
    mag     = data(:,6);

    if size(data,2) >= 7
        shock = data(:,7);
    else
        shock = double(abs(mag-1.0) > 2.0);
    end

    Fs            = 1/mean(diff(t));
    total_time    = t(end)-t(1);
    n_shocks      = sum(diff(shock)==1);   % so su kien shock
    max_shock_g   = max(mag) - 1.0;
    mean_temp     = mean(temp);
    max_temp      = max(temp);
    drift_yaw     = abs(yaw(end)-yaw(1));

    fprintf('\n--- TONG KET HE THONG ---\n');
    fprintf('  Thoi gian ghi:   %.1f s  |  Fs: %.1f Hz\n', total_time, Fs);
    fprintf('  So su kien shock: %d\n', n_shocks);
    fprintf('  Shock lon nhat:  %.4f g\n', max_shock_g);
    fprintf('  Nhiet do TB:     %.1f C  |  Max: %.1f C\n', mean_temp, max_temp);
    fprintf('  Drift Yaw:       %.2f deg trong %.1f s\n', drift_yaw, total_time);
    fprintf('  Roll range:      %.2f ~ %.2f deg\n', min(roll), max(roll));
    fprintf('  Pitch range:     %.2f ~ %.2f deg\n', min(pitch), max(pitch));

    % ---- Ve Dashboard ----
    fig = figure('Name','Bai 7 - Dashboard Tong Hop','NumberTitle','off',...
                 'Position',[50 50 1500 950]);
    sgtitle('Bai 7: Dashboard Giam sat IMU MPU6500 - Tong Hop','FontSize',15,'FontWeight','bold');

    % Tieu bieu tren: Roll, Pitch, Yaw
    subplot(4,3,[1 2]);
    plot(t, roll,'#1d3557','LineWidth',1.5,'DisplayName','Roll'); hold on;
    plot(t, pitch,'#e63946','LineWidth',1.5,'DisplayName','Pitch');
    xlabel('t (s)'); ylabel('Goc (deg)');
    title('Roll & Pitch (Kalman Filter)'); legend; grid on; hold off;

    subplot(4,3,3);
    plot(t, yaw,'#2a9d8f','LineWidth',1.5);
    xlabel('t (s)'); ylabel('Yaw (deg)');
    title(sprintf('Yaw (Gyro drift=%.2f deg)',drift_yaw)); grid on;

    % Bien do gia toc & shock
    subplot(4,3,[4 5]);
    area_t  = t;
    area_s  = (mag-1).*shock;
    yyaxis left;
    plot(t, mag-1, 'k','LineWidth',1,'DisplayName','|a|-1g'); hold on;
    area(area_t, area_s,'FaceColor','y','FaceAlpha',0.5,'DisplayName','Shock zone');
    yline(2.0,'r--','Label','Nguong 2g','LineWidth',1.5);
    ylabel('Gia toc (g)');
    yyaxis right;
    plot(t, shock,'r','LineWidth',1.5,'DisplayName','Shock flag');
    ylabel('Shock (0/1)');
    xlabel('t (s)'); title('Gia toc tong & Phat hien Shock');
    legend('Location','best'); grid on; hold off;

    % Nhiet do
    subplot(4,3,6);
    plot(t, temp,'#e76f51','LineWidth',1.5);
    yline(mean_temp,'k--','Label',sprintf('TB=%.1fC',mean_temp),'LineWidth',1.5);
    xlabel('t (s)'); ylabel('Nhiet do (C)');
    title(sprintf('Nhiet do cam bien | Max=%.1fC',max_temp)); grid on;

    % Histogram Roll & Pitch
    subplot(4,3,7);
    histogram(roll, 40,'FaceColor','#1d3557','EdgeColor','white','Normalization','probability');
    xlabel('Roll (deg)'); ylabel('Ti le'); title('Phan phoi Roll'); grid on;

    subplot(4,3,8);
    histogram(pitch, 40,'FaceColor','#e63946','EdgeColor','white','Normalization','probability');
    xlabel('Pitch (deg)'); ylabel('Ti le'); title('Phan phoi Pitch'); grid on;

    % Scatter Roll vs Pitch (quang dao goc)
    subplot(4,3,9);
    scatter(roll, pitch, 5, t,'filled','MarkerFaceAlpha',0.6);
    colorbar; colormap('jet');
    xlabel('Roll (deg)'); ylabel('Pitch (deg)');
    title('Hanh trinh goc Roll-Pitch (mau = thoi gian)'); grid on; axis equal;

    % FFT cua roll_kf
    subplot(4,3,10);
    N  = length(roll);
    f  = (0:floor(N/2))*Fs/N;
    Y  = abs(fft((roll-mean(roll)).*hanning(N)))*2/N;
    plot(f, Y(1:floor(N/2)+1),'#1d3557','LineWidth',1.2);
    [~,di] = max(Y(2:floor(N/2)+1));
    xline(f(di+1),'r--','Label',sprintf('%.2fHz',f(di+1)),'LineWidth',1.5);
    xlabel('Tan so (Hz)'); ylabel('Bien do'); title('FFT cua Roll'); grid on;
    xlim([0 min(Fs/2,20)]);

    % Nhiet do vs drift yaw (tuong quan nhiet)
    subplot(4,3,11);
    scatter(temp, abs(yaw-yaw(1)), 3, 'filled', 'MarkerFaceColor','#f4a261','MarkerFaceAlpha',0.5);
    xlabel('Nhiet do (C)'); ylabel('|Yaw drift| (deg)');
    title('Tuong quan nhiet do vs Yaw drift'); grid on;

    % Bang tong ket cuoi
    subplot(4,3,12);
    axis off;
    tbl = {
        'Thoi luong (s)',    sprintf('%.1f', total_time);
        'Fs (Hz)',           sprintf('%.1f', Fs);
        'So mau',            num2str(length(t));
        'So shock',          num2str(n_shocks);
        'Shock max (g)',     sprintf('%.4f', max_shock_g);
        'Nhiet do TB (C)',   sprintf('%.2f', mean_temp);
        'Nhiet do max (C)',  sprintf('%.2f', max_temp);
        'Roll range (deg)',  sprintf('[%.1f, %.1f]',min(roll),max(roll));
        'Pitch range (deg)', sprintf('[%.1f, %.1f]',min(pitch),max(pitch));
        'Yaw drift (deg)',   sprintf('%.2f',drift_yaw);
    };
    uitable(fig,'Data',tbl,'ColumnName',{'Thong so','Gia tri'},...
        'ColumnWidth',{145,100},'Units','normalized',...
        'Position',[0.68 0.01 0.30 0.22],'FontSize',8);

    save_figure(fig,'bai7_dashboard.png');
    fprintf('\n[OK] Bai 7 hoan tat. Da luu: bai7_dashboard.png\n\n');
end

%% ================================================================
%  HAM TIEN ICH
% ================================================================

% Doc file CSV, bo qua dong comment (#) va header chu
function data = load_csv(fname, min_cols)
    data = [];
    fid  = fopen(fname,'r');
    if fid == -1
        fprintf('[LOI] Khong mo duoc file: %s\n', fname);
        fprintf('      Dam bao file dat cung thu muc voi MPU6500_Lab.m\n');
        return;
    end

    rows = {};
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line) || line(1)=='#', continue; end
        parts = strsplit(line, {',','\t',' '});
        vals  = str2double(parts);
        % Bo qua dong co chu (header) hoac khong du cot
        if any(isnan(vals)) || length(vals) < min_cols, continue; end
        rows{end+1} = vals(1:min_cols); %#ok<AGROW>
    end
    fclose(fid);

    if isempty(rows)
        fprintf('[LOI] File %s khong co du lieu hop le (%d cot so).\n', fname, min_cols);
        return;
    end
    data = cell2mat(rows');
    fprintf('[OK] Doc %d mau tu: %s\n', size(data,1), fname);
end

% Luu figure ra file PNG
function save_figure(fig, fname)
    try
        exportgraphics(fig, fname, 'Resolution', 150);
    catch
        saveas(fig, fname);
    end
end
