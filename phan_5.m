clc; clear; close all;
%  PHAN 1: DOC VA XU LY DU LIEU SHOCK
fprintf('=== PHAN 1: SHOCK DETECTION ===\n');

fid = fopen('bai5_shock_data.txt','r');
time_ms=[]; Ax=[]; Ay=[]; Az=[]; Mag=[]; Shock_flag=[];
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if isempty(line), continue; end
    if ~(line(1)=='-' || (line(1)>='0' && line(1)<='9')), continue; end
    parts = strsplit(line, ',');
    if numel(parts) < 6, continue; end
    try
        time_ms(end+1)   = str2double(parts{1});
        Ax(end+1)        = str2double(parts{2});
        Ay(end+1)        = str2double(parts{3});
        Az(end+1)        = str2double(parts{4});
        Mag(end+1)       = str2double(parts{5});
        Shock_flag(end+1)= str2double(parts{6});
    catch
        continue;
    end
end
fclose(fid);
time_ms = time_ms(:); Ax=Ax(:); Ay=Ay(:); Az=Az(:);
Mag=Mag(:); Shock_flag=Shock_flag(:);

%% --- Phan tich tung su kien shock ---
shock_events = struct();
n_event = 0;
in_s = false;
t_start_idx = 0;

for i = 1:length(Shock_flag)
    if Shock_flag(i)==1 && ~in_s
        in_s = true; t_start_idx = i; n_event = n_event+1;
        shock_events(n_event).start_idx = i;
    end
    if Shock_flag(i)==0 && in_s
        in_s = false;
        shock_events(n_event).end_idx = i-1;
    end
end
if in_s, shock_events(n_event).end_idx = length(Shock_flag); end

fprintf('So su kien shock phat hien: %d\n', n_event);
fprintf('%-8s %-12s %-16s %-18s %-16s\n', ...
    'Shock#','t_bat_dau(ms)','Bien_do_dinh(g)','Thoi_gian_xung(ms)','Settling_time(ms)');

QUIET = 1.0;
QUIET_BAND = 0.05; 

for k = 1:n_event
    s = shock_events(k).start_idx;
    e = shock_events(k).end_idx;
    
    peak_g   = max(Mag(s:e));
    t_start  = time_ms(s);
    t_end    = time_ms(e);
    pulse_ms = t_end - t_start;
        settle_ms = NaN;
    for j = e+1:length(Mag)
        if abs(Mag(j) - QUIET) <= QUIET_BAND
            if j+9 <= length(Mag) && all(abs(Mag(j:j+9)-QUIET) <= QUIET_BAND)
                settle_ms = time_ms(j) - t_end;
                break;
            end
        end
    end
    
    shock_events(k).peak_g    = peak_g;
    shock_events(k).t_start   = t_start;
    shock_events(k).pulse_ms  = pulse_ms;
    shock_events(k).settle_ms = settle_ms;
        after = Mag(e+1:min(e+300,length(Mag)));
    t_after = time_ms(e+1:min(e+300,length(time_ms)));
    [~, locs] = findpeaks(after, 'MinPeakProminence', 0.02);
    if length(locs) >= 2
        T_osc = mean(diff(t_after(locs)))*2/1000;
        f_nat = 1/T_osc;
    else
        f_nat = NaN;
    end
    shock_events(k).f_nat = f_nat;
    
    fprintf('%-8d %-12.1f %-16.3f %-18.1f %-16.1f\n', ...
        k, t_start, peak_g, pulse_ms, settle_ms);
end

%% --- Figure 1: Toan bo tín hieu shock ---
figure('Name','Bai 5 - Shock Detection','NumberTitle','off','Position',[50 500 1100 400]);
hold on;
plot(time_ms, Mag, 'b-', 'LineWidth', 1, 'DisplayName','|a| (g)');
plot(time_ms, Az,  'Color',[0.2 0.7 0.2], 'LineWidth', 0.8, 'DisplayName','Az (g)');
for k = 1:n_event
    s = shock_events(k).start_idx;
    e = shock_events(k).end_idx;
    patch([time_ms(s) time_ms(e) time_ms(e) time_ms(s)], ...
          [0 0 8 8], 'r', 'FaceAlpha', 0.15, 'EdgeColor','none', 'HandleVisibility','off');
    text(time_ms(s)+1, shock_events(k).peak_g+0.3, ...
        sprintf('Shock %d\n%.2fg\n%dms', k, shock_events(k).peak_g, shock_events(k).pulse_ms), ...
        'FontSize', 8, 'Color', 'r');
end

SHOCK_THRESHOLD_MARKER = 2.0;
yline(1+SHOCK_THRESHOLD_MARKER, 'r--', 'LineWidth', 1.2, ...
    'Label', sprintf('Nguong (+%.1fg)', SHOCK_THRESHOLD_MARKER), 'HandleVisibility','off');
yline(1.0, 'k:', 'LineWidth', 1, 'Label', '1g', 'HandleVisibility','off');
xlabel('Thoi gian (ms)'); ylabel('Gia toc (g)');
title('Bai 5 - Shock Detection: Toan bo tin hieu');
legend('Location','northwest'); grid on; ylim([-1 8]);

%% --- Figure 2: Zoom vao tung su kien shock + settling ---
figure('Name','Bai 5 - Chi tiet Shock','NumberTitle','off','Position',[50 50 1100 500]);
for k = 1:n_event
    subplot(1, n_event, k);
    s = shock_events(k).start_idx;
    e = shock_events(k).end_idx;    
    i0 = max(1,   find(time_ms >= time_ms(s)-50, 1));
    i1 = min(length(time_ms), find(time_ms >= time_ms(e)+200, 1));
    if isempty(i1), i1 = length(time_ms); end
    
    t_win = time_ms(i0:i1);
    m_win = Mag(i0:i1);
    
    hold on;
    plot(t_win, m_win, 'b-', 'LineWidth', 1.2);
    patch([time_ms(s) time_ms(e) time_ms(e) time_ms(s)], ...
          [0 0 8 8], 'r', 'FaceAlpha', 0.2, 'EdgeColor','none');
    yline(1.0, 'k--', 'LineWidth', 1);
    yline(1.05, 'g:', 'LineWidth', 0.8);
    yline(0.95, 'g:', 'LineWidth', 0.8);
    
    xlabel('t (ms)'); ylabel('|a| (g)');
    title(sprintf('Shock %d: Dinh=%.2fg | Xung=%dms | Settle=%.0fms', ...
        k, shock_events(k).peak_g, shock_events(k).pulse_ms, shock_events(k).settle_ms));
    grid on;
end
sgtitle('Chi tiet tung su kien Shock');

%  PHAN 2: DOC DU LIEU FFT (512 mau, fs=500Hz)
fprintf('\n=== PHAN 2: FFT ANALYSIS ===\n');

fid2 = fopen('bai5_fft_data.txt','r');
vib_data = [];
in_block = false;

while ~feof(fid2)
    line = strtrim(fgetl(fid2));
    if strcmp(line, 'SAMPLE_START'), in_block = true; continue; end
    if strcmp(line, 'SAMPLE_END'),   in_block = false; break;  end
    if ~in_block, continue; end
    val = str2double(line);
    if ~isnan(val)
        vib_data(end+1) = val;
    end
end
fclose(fid2);

vib_data = vib_data(:);
N  = length(vib_data);
fs = 500;
fprintf('So mau doc duoc: %d  |  fs=%d Hz  |  T_total=%.3f s\n', N, fs, N/fs);

%% --- FFT tinh toan ---
Y     = fft(vib_data);
P2    = abs(Y/N);
P1    = P2(1:floor(N/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
f_axis = fs*(0:floor(N/2))/N;
[pks, locs_f] = findpeaks(P1, 'SortStr','descend', 'NPeaks', 5, 'MinPeakProminence', 0.005);
fprintf('\nTop 5 thanh phan tan so:\n');
fprintf('%-4s  %-12s  %-12s\n','#','Tan so (Hz)','Bien do (g)');
for i = 1:length(pks)
    fprintf('%-4d  %-12.1f  %-12.4f', i, f_axis(locs_f(i)), pks(i));
    if i==1, fprintf('  *** TAN SO CHU DAO ***'); end
    fprintf('\n');
end

dominant_f = f_axis(locs_f(1));

%% --- Figure 3: Tin hieu dao dong thoi gian ---
t_vib = (0:N-1)/fs;
figure('Name','Bai 5 - Vibration Signal','NumberTitle','off','Position',[600 500 900 350]);
plot(t_vib, vib_data, 'b-', 'LineWidth', 0.8);
xlabel('Thoi gian (s)'); ylabel('Gia toc (g, DC da tru)');
title(sprintf('Bai 5 - Tin hieu dao dong (%d mau, fs=%dHz)', N, fs));
grid on;

%% --- Figure 4: Pho FFT ---
figure('Name','Bai 5 - FFT Spectrum','NumberTitle','off','Position',[600 50 900 400]);
subplot(2,1,1);
stem(f_axis, P1, 'b', 'MarkerSize', 3);
hold on;
for i = 1:min(3,length(pks))
    stem(f_axis(locs_f(i)), P1(locs_f(i)), 'r', 'LineWidth', 2, 'MarkerSize', 6);
    text(f_axis(locs_f(i))+1, P1(locs_f(i)), ...
        sprintf('%.0fHz\n%.3fg', f_axis(locs_f(i)), P1(locs_f(i))), ...
        'FontSize', 8, 'Color','r');
end
xlabel('Tan so (Hz)'); ylabel('Bien do (g)');
title(sprintf('Pho FFT - Tan so chu dao: %.1f Hz', dominant_f));
xlim([0 250]); grid on;

subplot(2,1,2);
fc_lpf = 80;
[b_lpf, a_lpf] = butter(4, fc_lpf/(fs/2), 'low');
vib_filtered = filtfilt(b_lpf, a_lpf, vib_data);

Y_f  = fft(vib_filtered);
P2f  = abs(Y_f/N);
P1f  = P2f(1:floor(N/2)+1);
P1f(2:end-1) = 2*P1f(2:end-1);

hold on;
plot(f_axis, P1,  'b-',  'LineWidth', 1,   'DisplayName', 'Truoc LPF');
plot(f_axis, P1f, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Sau LPF (fc=%dHz)', fc_lpf));
xline(fc_lpf, 'k:', 'LineWidth', 1.2, 'Label', sprintf('fc=%dHz', fc_lpf));
xlabel('Tan so (Hz)'); ylabel('Bien do (g)');
title('So sanh Pho FFT: Truoc va Sau Low-Pass Filter');
legend('Location','northeast'); xlim([0 250]); grid on;

%  TONG KET KET QUA
fprintf('\n=== TONG KET BAI 5 ===\n');
fprintf('--- SHOCK ---\n');
for k = 1:n_event
    fprintf('  Shock #%d: Bien_do_dinh=%.3fg | Xung=%.0fms | Settling=%.0fms\n', ...
        k, shock_events(k).peak_g, shock_events(k).pulse_ms, shock_events(k).settle_ms);
    if ~isnan(shock_events(k).f_nat)
        fprintf('            Tan so dao dong tu nhien ~%.1f Hz\n', shock_events(k).f_nat);
    end
end
fprintf('--- FFT ---\n');
fprintf('  Tan so chu dao   : %.1f Hz (bien do %.4f g)\n', f_axis(locs_f(1)), P1(locs_f(1)));
if length(locs_f) >= 2
fprintf('  Bac 2 (harmonic) : %.1f Hz (bien do %.4f g)\n', f_axis(locs_f(2)), P1(locs_f(2)));
end
fprintf('  Hieu qua LPF (fc=%dHz): Loai cac thanh phan > %dHz\n', fc_lpf, fc_lpf);
fprintf('=== XONG ===\n');