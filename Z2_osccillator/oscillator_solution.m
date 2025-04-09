clc; clear; close all;

% step1 Load sinogram
load('z2_oscillator_comp.mat'); 
figure;
imagesc(theta, 1:size(sinogram,1), sinogram);
colormap gray;
colorbar;
xlabel('Projection Angle (degrees)');
ylabel('Detector Position');
title('Sinogram');
% step2 Define time (1 degree per second, 360 projections)
num_angles = size(sinogram, 2);
t_data = time;

% step3 Compute physical detector positions (unit: mm) ---
num_detectors = size(sinogram, 1);
detector_spacing = 1; 
center_pos = (num_detectors + 1) / 2;
detector_positions = ((1:num_detectors) - center_pos) * detector_spacing;  
% [-72, ..., +72] 

% step4: Compute weighted centroid of each projection column (in mm) ---
r_data = zeros(1, num_angles);  % projected positions
for i = 1:num_angles
    column = double(sinogram(:, i));  % ensure float
    weights = column;
    r_data(i) = sum(weights .* detector_positions') / sum(weights);
end

% step5: Center r_data to mean-zero for stability 
%r_data = r_data - mean(r_data);

% step6: Define harmonic projection model function 
model_fun = @(p, t) ...
    p(6) * cos(pi * t / 180) + p(7) * sin(pi * t / 180) + ...
    p(1) * sin(p(2) * t + p(3)) .* ( ...
        (p(4) / norm([p(4), p(5)])) * cos(pi * t / 180) + ...
        (p(5) / norm([p(4), p(5)])) * sin(pi * t / 180) );

% --- Step 7: Initial guess for parameters [A, omega, phi, u_x, u_y, b_x, b_y] ---
p0 = [20, 0.026, 0, 0.5, 1e5, 25, -10];

% --- Step 8: Fit model to data ---
options = optimoptions('lsqcurvefit','Display','iter','MaxIterations',1000);
[p_fit, resnorm] = lsqcurvefit(model_fun, p0, t_data, r_data, [], [], options);

% --- Step 9: Extract fitted parameters ---
A     = p_fit(1);
omega = p_fit(2);
phi   = p_fit(3);
u_raw = [p_fit(4), p_fit(5)];
u     = u_raw / norm(u_raw);
b     = [p_fit(6), p_fit(7)];

% --- Step 10: Evaluate model with fitted parameters ---
cos_theta = cos(pi * t_data / 180);
sin_theta = sin(pi * t_data / 180);
dot_u_n = u(1) * cos_theta + u(2) * sin_theta;
dot_b_n = b(1) * cos_theta + b(2) * sin_theta;
r_fit = dot_b_n + A * sin(omega * t_data + phi) .* dot_u_n;

% --- Step 11: Plot and compare ---
figure;
plot(t_data, r_data, 'b.', 'DisplayName', 'Measured centroid r(t)');
hold on;
plot(t_data, r_fit, 'r-', 'LineWidth', 2, 'DisplayName', 'Fitted r(t)');
xlabel('Time (s)');
ylabel('Projection position (mm)');
title('2D Harmonic Fit using Weighted Centroid (mm)');
legend;
grid on;

x_traj = b(1) + A * sin(omega * t_data + phi) * u(1);
y_traj = b(2) + A * sin(omega * t_data + phi) * u(2);

figure;
plot(x_traj, y_traj, 'r-', 'LineWidth', 2); hold on;
plot(b(1), b(2), 'bo', 'MarkerSize', 8, 'DisplayName', 'Center');
text(b(1) + 1, b(2), 'Center', 'Color', 'blue', 'FontSize', 10);
xlabel('x position (mm)');
ylabel('y position (mm)');
title('2D Trajectory of the Molecule with Center Marked');
legend('Trajectory', 'Center');
axis equal;
grid on;

% --- Step 12: Output fitted parameters and error ---
mse = mean((r_data - r_fit).^2);
fprintf('Fitting Results (Weighted Centroid, mm units):\n');
fprintf('Amplitude A: %.4f \n', A);
fprintf('Angular Frequency omega: %.6f rad/s\n', omega);
fprintf('Phase phi: %.4f rad\n', phi);
fprintf('Direction vector u (normalized): [%.4e, %.4e]\n', u(1), u(2));
fprintf('Original u vector: [%.4e, %.4e]\n', u_raw(1), u_raw(2));
fprintf('Center position b: [%.4f, %.4f] \n', b(1), b(2));
fprintf('Mean Squared Error (MSE): %.6f \n', mse);
