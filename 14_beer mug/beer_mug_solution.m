clc; clear; close all;

% Load data
load('14_beer_comp.mat');

% show sinogram
figure; 
imagesc(sinogram); 
colormap('gray'); 
colorbar; 
xlabel('Projection Index'); 
ylabel('Detector Index'); 
title('Sinogram Visualization'); 

% Choose detector indices
idx1 = 55;
idx2 = 105;

% Get projection values from degree 1 360 720
I0_1 = sinogram(idx1, 1);
I1_1 = sinogram(idx1, 360);
I2_1 = sinogram(idx1, 720);

I0_2 = sinogram(idx2, 1);
I1_2 = sinogram(idx2, 360);
I2_2 = sinogram(idx2, 720);

% Compute percentage decreases
P1_half = (I0_1 - I1_1) / I0_1 * 100;
P1_last = (I1_1 - I2_1) / I0_1 * 100;
P1_full = (I0_1 - I2_1) / I0_1 * 100;

P2_half = (I0_2 - I1_2) / I0_2 * 100;
P2_last = (I1_2 - I2_2) / I0_2 * 100;
P2_full = (I0_2 - I2_2) / I0_2 * 100;

% Speeds
S1_half = P1_half / 5;
S1_last = P1_last / 5;
S1_full = P1_full / 10;

S2_half = P2_half / 5;
S2_last = P2_last / 5;
S2_full = P2_full / 10;

% Print results
fprintf('Drinker 1 (0–5s):   %.2f %%/sec\n', S1_half);
fprintf('Drinker 1 (5–10s):  %.2f %%/sec\n', S1_last);
fprintf('Drinker 1 (0–10s):  %.2f %%/sec\n', S1_full);
fprintf('Drinker 2 (0–5s):   %.2f %%/sec\n', S2_half);
fprintf('Drinker 2 (5–10s):  %.2f %%/sec\n', S2_last);
fprintf('Drinker 2 (0–10s):  %.2f %%/sec\n', S2_full);