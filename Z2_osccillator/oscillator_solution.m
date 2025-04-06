clc; clear; close all;

% Load data
load('z2_oscillator_comp.mat');

% sinogram 
figure;
subplot(2,1,1);
imagesc(theta, 1:size(sinogram,1), sinogram);
colormap gray;
colorbar;
xlabel('Angle (degrees)');
ylabel('Detector index');
title('Sinogram');

% Filtered Back Projection
rec = iradon(sinogram, theta, 'linear', 'Ram-Lak', 1.0, nSize);
subplot(2,1,2);
imagesc(rec);
colormap gray;
axis image;
title('Reconstructed Image');

%% === 提取轨迹（几何方法） ===
[detector_len, num_angles] = size(sinogram);
R = detector_len / 2;
theta_rad = deg2rad(theta);
s_idx = vec2ind(sinogram); % 每列最大值的位置
s_vals = s_idx - R;        % 投影值 s_i（已居中）

A = [cos(theta_rad'); sin(theta_rad')'];  % 系数矩阵，每行为 [cosθ sinθ]
A = A';  % (360x2)
pos = zeros(num_angles, 2);  % 存储[x, y]位置

for i = 1:num_angles
    Ai = A(i, :);
    si = s_vals(i);
    pos(i,:) = (Ai' * si) / (Ai * Ai');  % 最小二乘解：x = A^T * s / (A*A^T)
end

x_pos = pos(:,1);
y_pos = pos(:,2);

%% === 拟合简谐模型 r(t) = A * sin(ωt + φ) + b ===
harmonic = @(p, t) p(1) * sin(p(2)*t + p(3)) + p(4);
loss = @(p) sum((harmonic(p, time) - y_pos).^2);

% 初始猜测: A, omega, phi, b
p0 = [1, 0.1, 0, 0];
opt = optimset('Display','off');
params = fminsearch(loss, p0, opt);
fitted = harmonic(params, time);

%% === 画轨迹拟合 ===
figure;
plot(time, y_pos, 'b-', 'LineWidth', 2); hold on;
plot(time, fitted, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Y Position (pixels)');
legend('Observed', 'Fitted');
title('Fitted Harmonic Oscillator');
grid on;

%% === 输出拟合参数 ===
fprintf('Fitted model: r(t) = A * sin(omega * t + phi) + b\n');
fprintf('A = %.4f\n', params(1));
fprintf('omega = %.4f rad/s → T = %.2f s\n', params(2), 2*pi/params(2));
fprintf('phi = %.4f rad\n', params(3));
fprintf('b = %.4f\n', params(4));

%% === 辅助函数：找每列最大值下标 ===
function idx = vec2ind(M)
    [~, idx] = max(M, [], 1);
end

