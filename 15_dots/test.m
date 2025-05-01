clc;clear;close all;
% Step 1: 构造一个空白图像
imgSize = 100; % 图像大小
img = zeros(imgSize, imgSize);

% Step 2: 选取4个像素点（可自选）
% points = [69 68; 70 29; 36 57; 23 8;9 12]; % [row col] 格式
points = [69 68; 70 29; 23 8;9 12];
% Step 3: 标记这些点为1（注意是 row, col）
for i = 1:size(points,1)
    img(points(i,1), points(i,2)) = 1;
end

% Step 4: 显示原始图像
figure;
subplot(1,3,1);
imshow(img, []);
title('Original Point Image');

% Step 5: 前向投影（Radon变换）
theta = [19,75,139]; % 投影角度
[sinogram, xp] = radon(img, theta);

% Step 6: 显示 sinogram
subplot(1,3,2);
imagesc(theta, xp, sinogram);
colormap gray;
xlabel('\\theta (degrees)');
ylabel('Projection position');
title('Sinogram');

% Step 7: 反投影重建图像
recon = iradon(sinogram, theta, 'linear', 'Ram-Lak', 1.0, imgSize);

% Step 8: 显示重建图像
subplot(1,3,3);
imshow(recon, []);
title('Reconstructed Image');
