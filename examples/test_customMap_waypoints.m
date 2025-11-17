% Testing script with a custom map (createTest3DMap) and trying single
% waypoint target (3D), multiple waypoint target (3D x,y,z and 4D
% x,y,z,heading)
clear; close all; clc;

setupPath();

rng(1,"twister");

plotTree = true;

[origMap, smoothMap, planMap, mapLimits] = createTest3DMap(20, 35);

limits = [mapLimits; 0 2*pi];

start  = [ 0 0 60 ];
wpt1 = [ 110 360 65 ];
wpt2 = [ 420 340 80 ];
goal = [ 440 650 60 ];

N = 5000; % number of random samples
rn = optimalRadius(N+2, limits(1:3,:)); % radius to find neighbors
goalRadius = 10; % radius of the goal area around the goal wpt

MaxRollAngle         = 50*pi/180;
AirSpeed             = 20;
FlightPathAngleLimit = [-45, 45]*pi/180;

flightParams = [MaxRollAngle, AirSpeed, FlightPathAngleLimit];

%% Single Waypoint target - 3D Problem (x,y,z)
tic;
wpts = [start; goal];
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

% Plot the tree if the nodes are less than 5000, otherwise is too much
E = E{1};
V = V{1};
if plotTree && size(E,1) <= 5000
    figure;
    show(origMap); alpha(0.3);
    hold on;
    
    % Extract endpoints
    p1 = V(E(:,1), :);   % parents
    p2 = V(E(:,2), :);   % childs
    
    % Build 2×P arrays for line()
    X = [p1(:,1)'; p2(:,1)'];
    Y = [p1(:,2)'; p2(:,2)'];
    Z = [p1(:,3)'; p2(:,3)'];
    
    % Draw all segments in one call
    line(X, Y, Z, ...
         'Color',    [0.7 0.7 0.7], ...  % light gray
         'LineWidth', 0.5);
    
    % Plot solution path on top
    plot3(traj(:,1), traj(:,2), traj(:,3), 'r-', 'LineWidth',2);
    
    scatter3(traj(1,1),traj(1,2),traj(1,3),100,'go','filled');
    scatter3(traj(end,1),traj(end,2),traj(end,3),100,'mo','filled');
    scatter3(goal(1), goal(2), goal(3),150,'b*','LineWidth',2);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    view(-115,60); axis equal
    title('FMT* tree and final path');

    clear p1 p2 X Y Z E V
end

% Smooth the trajectory
tic;
smoothedTraj = smoothTrajectory( flightParams, limits, smoothMap, traj, wpts, goalRadius);
elapsed = toc;     % elapsed time (in seconds)
fprintf('Smoothing Elapsed time: %.4f s\n', elapsed);

% Plot the smoothed trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
hTraj = plot3(smoothedTraj(:,1), smoothedTraj(:,2), smoothedTraj(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(smoothedTraj(1,1),smoothedTraj(1,2),smoothedTraj(1,3), 100, 'go','filled');
hReached = scatter3(smoothedTraj(end,1),smoothedTraj(end,2),smoothedTraj(end,3), 100, 'mo','filled');
hGoal = scatter3(goal(1), goal(2), goal(3), 150, 'b*','LineWidth',2);

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

%% Multiple waypoint targetting - 3D Problem (x,y,z)
tic;
wpts = [start; wpt1; wpt2; goal];
[trajAll, ~, ~] = FMTWaypoints(planMap, limits(1:3,:), wpts(:, 1:3), N, rn, goalRadius);
elapsed = toc;     % elapsed time (in seconds)
fprintf('FMT Elapsed time: %.4f s\n', elapsed);

% Plot the trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
hTraj = plot3(trajAll(:,1), trajAll(:,2), trajAll(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(trajAll(1,1),trajAll(1,2),trajAll(1,3), 100, 'go','filled');
hReached = scatter3(trajAll(end,1),trajAll(end,2),trajAll(end,3), 100, 'mo','filled');
hGoal = scatter3(goal(1), goal(2), goal(3), 150, 'b*','LineWidth',2);

plot3( [trajAll(end,1), goal(1)], ...
       [trajAll(end,2), goal(2)], ...
       [trajAll(end,3), goal(3)], ...
       'k--', 'LineWidth',1);

% annotate the distance to goal
distToGoal = norm(trajAll(end,1:3) - goal(1:3));
text( goal(1), goal(2), goal(3), ...
      sprintf('  %.2f m', distToGoal), ...
      'FontSize',12, 'Color','b' );

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(-115,60); axis equal
legend([hTraj, hStart, hReached, hGoal], ...
       {'Trajectory','Start','Reached z','Goal'}, ...
       'Location','best');
title(sprintf('FMT* solution (error to goal = %.2f m)', distToGoal));

% Smooth the trajectory
tic;
smoothedTraj = smoothTrajectory( flightParams, limits, smoothMap, trajAll, wpts, goalRadius);
elapsed = toc;     % elapsed time (in seconds)
fprintf('Smoothing Elapsed time: %.4f s\n', elapsed);

% Plot the smoothed trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
hTraj = plot3(smoothedTraj(:,1), smoothedTraj(:,2), smoothedTraj(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(smoothedTraj(1,1),smoothedTraj(1,2),smoothedTraj(1,3), 100, 'go','filled');
hReached = scatter3(smoothedTraj(end,1),smoothedTraj(end,2),smoothedTraj(end,3), 100, 'mo','filled');
hGoal = scatter3(goal(1), goal(2), goal(3), 150, 'b*','LineWidth',2);

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

%% Multiple waypoint targetting - 4D Problem (x,y,z,heading)
tic;
wpts = [start; wpt1; wpt2; goal];
wpts(:,4) = computeHeading(wpts);

g = 9.81;
w = AirSpeed^2/(g*tan(MaxRollAngle));
rn = optimalRadius(N+2, limits, w);
goalRadius = rn / 5;

[trajAll, ~, ~] = FMTWaypoints(planMap, limits, wpts, N, rn, goalRadius, w);
elapsed = toc;     % elapsed time (in seconds)
fprintf('FMT Elapsed time: %.4f s\n', elapsed);

% Plot the trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
hTraj = plot3(trajAll(:,1), trajAll(:,2), trajAll(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(trajAll(1,1),trajAll(1,2),trajAll(1,3), 100, 'go','filled');
hReached = scatter3(trajAll(end,1),trajAll(end,2),trajAll(end,3), 100, 'mo','filled');
hGoal = scatter3(goal(1), goal(2), goal(3), 150, 'b*','LineWidth',2);

plot3( [trajAll(end,1), goal(1)], ...
       [trajAll(end,2), goal(2)], ...
       [trajAll(end,3), goal(3)], ...
       'k--', 'LineWidth',1);

% annotate the distance to goal
distToGoal = norm(trajAll(end,1:3) - goal(1:3));
text( goal(1), goal(2), goal(3), ...
      sprintf('  %.2f m', distToGoal), ...
      'FontSize',12, 'Color','b' );

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(-115,60); axis equal
legend([hTraj, hStart, hReached, hGoal], ...
       {'Trajectory','Start','Reached z','Goal'}, ...
       'Location','best');
title(sprintf('FMT* solution (error to goal = %.2f m)', distToGoal));

% Smooth the trajectory
tic;
smoothedTraj = smoothTrajectory( flightParams, limits, smoothMap, trajAll, wpts, goalRadius);
elapsed = toc;     % elapsed time (in seconds)
fprintf('Smoothing Elapsed time: %.4f s\n', elapsed);

% Plot the smoothed trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
hTraj = plot3(smoothedTraj(:,1), smoothedTraj(:,2), smoothedTraj(:,3), 'r-', 'LineWidth', 2);
hStart = scatter3(smoothedTraj(1,1),smoothedTraj(1,2),smoothedTraj(1,3), 100, 'go','filled');
hReached = scatter3(smoothedTraj(end,1),smoothedTraj(end,2),smoothedTraj(end,3), 100, 'mo','filled');
hGoal = scatter3(goal(1), goal(2), goal(3), 150, 'b*','LineWidth',2);

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