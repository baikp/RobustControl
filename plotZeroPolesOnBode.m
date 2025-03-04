function plotZeroPolesOnBode(sys)
Ts = sys.Ts;
[z, p, k] = tf2zp(sys.Numerator{1}, sys.Denominator{1}); % 获取零极点

[mag, phase, w] = bode(sys); % 计算 Bode 图数据
mag = squeeze(20*log10(mag)); % 转换为 dB
phase = squeeze(phase); % 提取相位信息



figure;
subplot(2,1,1)
semilogx(w, mag, 'b'); % 幅频曲线
hold on
grid on
title('Bode Plot with Poles and Zeros')
xlabel('Frequency (rad/s)')
ylabel('Magnitude (dB)')

% 标注零点
for i = 1:length(z)
    wz = abs(log(z(i))/Ts); % 取零点的频率
    if wz > 0
        plot(wz, interp1(w, mag, wz), 'ro', 'MarkerSize', 8, 'LineWidth', 2)
        text(wz, interp1(w, mag, wz), sprintf('Zero: %.2f', wz), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right')
    end
end

% 标注极点
for i = 1:length(p)
    wp = abs(log(p(i))/Ts); % 取极点的频率
    if wp > 0
        plot(wp, interp1(w, mag, wp), 'bx', 'MarkerSize', 8, 'LineWidth', 2)
        text(wp, interp1(w, mag, wp), sprintf('Pole: %.2f', wp), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'right')
    end
end

subplot(2,1,2)
semilogx(w, phase, 'b'); % 相频曲线
hold on
grid on
xlabel('Frequency (rad/s)')
ylabel('Phase (degrees)')

% 在相位曲线上也标注零极点
for i = 1:length(z)
    wz = abs(log(z(i))/Ts); % 取零点的频率
    if wz > 0
        plot(wz, interp1(w, phase, wz), 'ro', 'MarkerSize', 8, 'LineWidth', 2)
    end
end

for i = 1:length(p)
    wp = abs(log(p(i))/Ts); % 取极点的频率
    if wp > 0
        plot(wp, interp1(w, phase, wp), 'bx', 'MarkerSize', 8, 'LineWidth', 2)
    end
end
end