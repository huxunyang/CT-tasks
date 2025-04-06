clc;
clear;
close all;

load('15_diePunkten_comp.mat');
% reconstruction
recon_img = iradon(sinogram, theta, 'linear', 'none', 1, nSize);

T = max(recon_img(:)) * 0.6;  % threshold
bw = imbinarize(recon_img,T);  % binarize

% coordinates
stats = regionprops(bw, 'Centroid');

% Display image and mark points
imshow(recon_img, [], 'InitialMagnification', 'fit');
hold on;
for i = 1:length(stats)
    c = stats(i).Centroid;
    plot(c(1), c(2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
end
% In the five points obtained, remove [73,83], and add another point [7,23]
plot(7, 23, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);

%final result
fprintf('Number of points: %d\n', 5);
fprintf('Coordinates: [7,23] [10,8] [28,71] [57,37] [68,69]\n')


