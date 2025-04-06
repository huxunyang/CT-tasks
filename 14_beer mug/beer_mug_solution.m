clc; 
clear; 
close all;

load('14_beer_comp.mat');

% show sinogram
figure; 
imagesc(sinogram); 
colormap('gray'); 
colorbar; 
xlabel('Projection Index'); 
ylabel('Detector Position'); 
title('Sinogram Visualization'); 

% reconstruction
rec = iradon(sinogram, theta, 'linear', 'Ram-Lak', 1.0, nSize);
figure;
imshow(rec, []);
title('Reconstructed Beer Cup');
colorbar;

% separate two drinkers
num_rows = size(sinogram, 1);
midpoint = floor(num_rows / 2);  % Ensure integer index

% Extract sinogram data for each drinker at the first and last time steps
initial_sinogram_1 = sinogram(1:midpoint, 1);   % Drinker 1 at t=0
final_sinogram_1 = sinogram(1:midpoint, end);   % Drinker 1 at last time

initial_sinogram_2 = sinogram(midpoint+1:end, 1); % Drinker 2 at t=0
final_sinogram_2 = sinogram(midpoint+1:end, end); % Drinker 2 at last time

% Compute average intensity for each drinker
initial_intensity_1 = mean(initial_sinogram_1);
final_intensity_1 = mean(final_sinogram_1);

initial_intensity_2 = mean(initial_sinogram_2);
final_intensity_2 = mean(final_sinogram_2);

% Calculate remaining beer percentage
final_beer_1 = (final_intensity_1 / initial_intensity_1) * 100;
final_beer_2 = (final_intensity_2 / initial_intensity_2) * 100;

% Calculate beer consumed by each drinker
beer_consumed_1 = 100 - final_beer_1;
beer_consumed_2 = 100 - final_beer_2;

% Calculate the average drinking speed in % per second (over 10 seconds)
time_elapsed = 10;  % Total time in seconds

speed_1 = beer_consumed_1 / time_elapsed;  % Drinker 1 speed
speed_2 = beer_consumed_2 / time_elapsed;  % Drinker 2 speed

% Display results
fprintf('Drinker 1 Final Beer Volume: %.2f%%\n', final_beer_1);
fprintf('Drinker 2 Final Beer Volume: %.2f%%\n', final_beer_2);
fprintf('Drinker 1 Drinking Speed: %.2f%% per second\n', speed_1);
fprintf('Drinker 2 Drinking Speed: %.2f%% per second\n', speed_2);

