% Testing script with a real map (buildOccupancyMap) and trying multiple
% waypoint target (3D x,y,z)

clear; close all; clc;

setupPath();

rng(1,"twister");

result = buildOccupancyMap("data/osm/boston.osm", 0.1, 20, 35);
origMap = result.maps{1}; 
smoothMap = result.maps{2};
planMap = result.maps{3};
mapLimits = result.mapLimits;

limits = [mapLimits; 0 2*pi];

kts2ms = 1/1.94384001;
MaxRollAngle         = 50*pi/180;
AirSpeed             = 120*kts2ms;
FlightPathAngleLimit = [-45, 45]*pi/180;

flightParams = [MaxRollAngle, AirSpeed, FlightPathAngleLimit];

%% Multiple Waypoint target

start = [ 920 -130 150 ];
wpt1  = [ 150 500 200 ];
goal = [ -600 -800 120 ];

N = 10000; % number of random samples
rn = optimalRadius(N, limits(1:3,:)); % radius to find neighbors
goalRadius = 10; % radius of the goal area around the goal wpt

tic;
wpts = [start; wpt1; goal];
[traj, E, V] = FMTWaypoints(planMap, limits(1:3,:), wpts(:, 1:3), N, rn, goalRadius);
elapsed = toc;     % elapsed time (in seconds)
fprintf('FMT Elapsed time: %.4f s\n', elapsed);

% Plot the trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
hTraj = plot3(traj(:,1), traj(:,2), traj(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(traj(1,1),traj(1,2),traj(1,3), 100, 'go','filled');
hReached = scatter3(traj(end,1),traj(end,2),traj(end,3), 100, 'mo','filled');
hGoal = scatter3(goal(1), goal(2), goal(3), 150, 'b*','LineWidth',2);

plot3( [traj(end,1), goal(1)], ...
       [traj(end,2), goal(2)], ...
       [traj(end,3), goal(3)], ...
       'k--', 'LineWidth',1);

% annotate the distance to goal
distToGoal = norm(traj(end,1:3) - goal(1:3));
text( goal(1), goal(2), goal(3), ...
      sprintf('  %.2f m', distToGoal), ...
      'FontSize',12, 'Color','b' );

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(-115,60); axis equal
legend([hTraj, hStart, hReached, hGoal], ...
       {'Trajectory','Start','Reached z','Goal'}, ...
       'Location','best');
title(sprintf('FMT* solution (error to goal = %.2f m)', distToGoal));

%% Smooth the trajectory
% If we want to target the waypoint in the middle of the trajectory (wpt1)
% we should pass as a parameter wpts, if we only want to target the goal
% and smooth the planned trajectory we should pass wpts([1 end],:)
tic;
smoothedTraj = smoothTrajectory( flightParams, limits, smoothMap, traj, wpts([1 end],:), goalRadius);
elapsed = toc;     % elapsed time (in seconds)
fprintf('Smoothing Elapsed time: %.4f s\n', elapsed);

% Plot the smoothed trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
hTraj = plot3(smoothedTraj(:,1), smoothedTraj(:,2), smoothedTraj(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(smoothedTraj(1,1),smoothedTraj(1,2),smoothedTraj(1,3), 100, 'go','filled');
hReached = scatter3(smoothedTraj(end,1),smoothedTraj(end,2),smoothedTraj(end,3), 100, 'mo','filled');
hGoal = scatter3(wpts(2:end,1), wpts(2:end,2), wpts(2:end,3), 150, 'b*','LineWidth',2);

plot3( [smoothedTraj(end,1), goal(1)], ...
       [smoothedTraj(end,2), goal(2)], ...
       [smoothedTraj(end,3), goal(3)], ...
       'k--', 'LineWidth',1);

% annotate the distance to goal
distToGoal = norm(smoothedTraj(end,1:3) - goal(1:3));
text( goal(1), goal(2), goal(3), ...
      sprintf('  %.2f m', distToGoal), ...
      'FontSize',12, 'Color','b' );

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(-115,60); axis equal
legend([hTraj, hStart, hReached, hGoal], ...
       {'Trajectory','Start','Reached z','Goal'}, ...
       'Location','best');
title(sprintf('FMT* smoothed solution (error to goal = %.2f m)', distToGoal));

