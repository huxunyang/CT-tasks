clc;clear;close all;

% load data
load('15_diePunkten_comp.mat');

% show sinogram
% figure;
% imagesc(sinogram);
% colormap gray;
% title('Sinogram');
% xlabel('Angle Index');
% ylabel('Detector Position');
% colorbar;

% projections at 3 angles
figure;
hold on;
for i = 1:length(theta)
%     subplot(3,1,i);
    plot(sinogram(:, i), 'DisplayName', ['\theta =' num2str(theta(i))]);
%     legend show;
%     title(sprintf('Projection at \\theta = %d^\\circ', theta(i)));
%     xlabel('Detector Index');
%     ylabel('Projection Value');
%     grid on;
end
legend show;
title('Projection curves at the different angles');
xlabel('Detector Index');
ylabel('Projection Value');
grid on;

% find peaks at each angle
for i = 1:size(sinogram, 2)
    proj = sinogram(:, i);
    [pks, locs] = findpeaks(proj, 'MinPeakHeight', 0.5 * max(proj));
    disp(['Angle ' num2str(theta(i)) ': Peaks at ' num2str(locs')]);
    peaks{i} = locs'
end
peak1 = peaks{1};   % theta = theta(1)
peak2 = peaks{2};  % theta = theta(2)
peak3 = peaks{3};  % theta = theta(3)

% all combos
all_combinations = combvec(peak1, peak2, peak3)';
nSize = double(nSize); 
diag_len = sqrt(2) * nSize;

% map detector index to [-diag/2, diag/2]
map_to_s = @(idx) (idx / size(sinogram,1)) * diag_len - diag_len/2;



points = [];
for i = 1:size(all_combinations,1)
    s1 = map_to_s(all_combinations(i,1));
    s2 = map_to_s(all_combinations(i,2));
    s3 = map_to_s(all_combinations(i,3));
    
    % line: A*x + B*y + C = 0
    L1 = [cosd(theta(1)), sind(theta(1)), -s1];
    L2 = [cosd(theta(2)), sind(theta(2)), -s2];
    L3 = [cosd(theta(3)), sind(theta(3)), -s3];
    
    p12 = intersect_lines(L1, L2);
    p23 = intersect_lines(L2, L3);
    p13 = intersect_lines(L1, L3);
    
    if ~isempty(p12) && ~isempty(p23) && ~isempty(p13)
        if norm(p12 - p23) < 2 && norm(p12 - p13) < 2
            points = [points; p12'];
        end
    end
end

% Remove duplicate points
final_points = [];
for i = 1:size(points,1)
    pt = points(i,:);
    if isempty(final_points) || all(vecnorm(final_points - pt, 2, 2) > 1)
        final_points = [final_points; pt];
    end
end

disp('Recovered coordinates (image-centered): ');
disp(final_points);

% convert to image coords
center = nSize / 2;
pixel_coords = round([center - final_points(:,2), center + final_points(:,1)]);  % [row, col]

disp('Pixel coordinates (row, col): ');
disp(pixel_coords);

% plot the dots on image
I = zeros(nSize);
for i = 1:size(pixel_coords,1)
    r = pixel_coords(i,1);
    c = pixel_coords(i,2);
    if r >= 1 && r <= nSize && c >= 1 && c <= nSize
        I(r, c) = 1;
    end
end
figure;
imshow(I);

% Forward projection
theta_rad = double(theta);
[~, xp] = radon(I, theta_rad);
sinogram_sim = radon(I, theta_rad);

% original vs simulated sinogram
figure;
subplot(1,2,1);
imagesc(sinogram);
colormap gray;
title('Original Sinogram');

subplot(1,2,2);
imagesc(sinogram_sim);
colormap gray;
title('Simulated Forward Projection');

% function: compute intersections
function pt = intersect_lines(L1, L2)
    A = [L1(1), L1(2); L2(1), L2(2)];
    b = [-L1(3); -L2(3)];
    if abs(det(A)) < 1e-6
        pt = [];
    else
        pt = A \ b;
    end
end
