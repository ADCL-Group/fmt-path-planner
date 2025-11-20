clc; clear; close all
result = buildOccupancyMap("data/osm/boston.osm", 0.1, 20, 35);
map3D = result.maps{1}; 

% Visualize the 3D Occupancy Map
figure;
show(map3D);
title('3D Occupancy Map from OSM Buildings');

% % Downsample: keep only every Nth point
% N = 1;  % Change this to control density (higher = faster, less detail)
% downsampledPoints = allOccupied(1:N:end, :);
% 
% % Display downsampled point cloud
% figure;
% pcshow(downsampledPoints, 'MarkerSize', 50);
% title('Downsampled 3D Occupancy Map');
% xlabel('X'); ylabel('Y'); zlabel('Z');


