clc;clear;close all;
% Step 1 Shepp-Logan phantom
phantom_size = 256;
img = phantom(phantom_size);

% Step 2: put in an area of metal with high attenuation
[x, y] = meshgrid(1:phantom_size, 1:phantom_size);
center = phantom_size / 2;
radius = 10;
metal_mask = ((x - center).^2 + (y - center).^2) < radius^2;
img(metal_mask) = 10;  % very large value

% Step 3 radon
theta = 0:1:179;
sinogram = radon(img, theta);

% Step 4: show sinogram
figure; imagesc(sinogram); colormap gray; title('Sinogram with Metal Artifacts');

% Step 5 Reconstruction using FBP
recon = iradon(sinogram, theta, 'linear', 'Ram-Lak', 1.0, phantom_size);

% Step 6: show result
figure;
imshow(recon, []); title('Reconstructed Image with Metal Artifacts');
