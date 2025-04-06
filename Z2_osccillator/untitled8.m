clc; clear; close all;

%% === 加载数据 ===
load('z2_oscillator_comp.mat');

%% === 提取 s_i（每列最大值索引）===
[detector_len, num_angles] = size(sinogram);
R = detector_len / 2;
[~, s_idx] = max(sinogram, [], 1);
s_vals = s_idx - R;  % 中心化：投影距离

theta_rad = deg2rad(theta);
A = [cos(theta_rad), sin(theta_rad)];
s_vals = (s_idx - R)';  % 别忘了转置成列向量

pos = A .* s_vals;  % 720 x 2，直接得到每帧的位置
x = pos(:,1);
y = pos(:,2);

%% === PCA/SVD：找出主运动方向 ===
mean_pos = mean(pos);
centered = pos - mean_pos;  % 数据中心化

[U, S, V] = svd(centered, 'econ');
principal_axis = V(:,1);  % 主方向单位向量（列）

% 将轨迹投影到主方向上（得到1D简谐运动轨迹）
proj = centered * principal_axis;  % 360 x 1

%% === 拟合简谐模型 r(t) = A sin(ω t + φ) + b ===
harmonic = @(p, t) p(1) * sin(p(2)*t + p(3)) + p(4);
loss = @(p) sum((harmonic(p, time) - proj).^2);
params0 = [1, 0.1, 0, 0];  % 初始值
params = fminsearch(loss, params0);
fitted = harmonic(params, time);

%% === 可视化：原始轨迹 + 主轴方向 ===
figure;
plot(x, y, 'b.'); hold on;
quiver(mean_pos(1), mean_pos(2), principal_axis(1), principal_axis(2), 10, 'r', 'LineWidth', 2);
axis equal;
title('Trajectory and Principal Motion Direction');
xlabel('x'); ylabel('y');
legend('Trajectory', 'Principal Direction');

%% === 可视化：主轴投影随时间变化 + 拟合 ===
figure;
plot(time, proj, 'b', 'LineWidth', 2); hold on;
plot(time, fitted, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Displacement along principal axis');
title('Fitted Harmonic Motion along Principal Direction');
legend('Observed', 'Fitted');
grid on;

%% === 输出拟合结果 ===
fprintf('\nFitted motion along principal direction:\n');
fprintf('A     = %.4f\n', params(1));
fprintf('omega = %.4f rad/s → T ≈ %.2f s\n', params(2), 2*pi/params(2));
fprintf('phi   = %.4f rad\n', params(3));
fprintf('b     = %.4f\n', params(4));
fprintf('Principal direction vector: [%.4f, %.4f]\n', principal_axis(1), principal_axis(2));
