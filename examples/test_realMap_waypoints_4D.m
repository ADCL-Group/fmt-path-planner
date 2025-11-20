% Testing script with a real map (buildOccupancyMap) and trying multiple 
% waypoint target (4D x,y,z,heading)

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
MaxRollAngle         = 35*pi/180;
AirSpeed             = 100*kts2ms;
FlightPathAngleLimit = [-45, 45]*pi/180;

flightParams = [MaxRollAngle, AirSpeed, FlightPathAngleLimit];

%% Multiple Waypoint target

start = [ 828 7 155 ];
wpt1  = [ 694 200 170 ];
wpt2  = [ 453 595 205 ];
wpt3  = [ 100 641 222 ];
wpt4  = [ -290 590 225 ];
wpt5  = [ -440 250 200 ];
goal = [ -600 -600 120 ];

g = 9.81;
w = AirSpeed^2/(g*tan(MaxRollAngle));

N = 5000; % number of random samples
rn = optimalRadius(N+2, limits, w); % radius to find neighbors
goalRadius = rn/2; % radius of the goal area around the goal wpt

tic;
wpts = [start; wpt1; wpt2; wpt3; wpt4; wpt5; goal];
wpts(:,4) = computeHeading(wpts);

% If it doesnt connect its because some points in the trajectory
% interpolation to check the collision might be outside the map so they are
% marked as unknown and they return a invalid
[traj, ~, ~] = FMTWaypoints(planMap, limits, wpts([1 end],:), N, rn, goalRadius, w, flightParams);
elapsed = toc;     % elapsed time (in seconds)
fprintf('FMT Elapsed time: %.4f s\n', elapsed);

conn = uavDubinsConnection( ...
            'MaxRollAngle',flightParams(1), ...
            'AirSpeed',flightParams(2), ...
            'FlightPathAngleLimit',flightParams(3:4));

% Plot the trajectory in the map
figure;
show(origMap); alpha(0.3);
hold on;
for k = 1:size(traj,1)-1
    % Compute the Dubins segment between traj(k,:) and traj(k+1,:)
    dubSeg = connect(conn, traj(k,:), traj(k+1,:));

    if isempty(dubSeg) || isnan(dubSeg{1}.Length)
        continue  % Skip invalid connections
    end

    % Sample the path with fine resolution
    dist = dubSeg{1}.Length;
    samplePoints = linspace(0, dist, 50);
    states = interpolate(dubSeg{1}, samplePoints);

    % Plot this segment
    hTraj = plot3(states(:,1), states(:,2), states(:,3), 'r-', 'LineWidth', 2);
end
hStart = scatter3(traj(1,1),traj(1,2),traj(1,3), 100, 'go','filled');
hReached = scatter3(traj(end,1),traj(end,2),traj(end,3), 100, 'mo','filled');
hGoal = scatter3(wpts(2:end,1), wpts(2:end,2), wpts(2:end,3), 150, 'b*','LineWidth',2);

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

% 1) Pull out positions and headings
X   = traj(:,1);
Y   = traj(:,2);
Z   = traj(:,3);
Psi = traj(:,4);      % heading in radians

% 2) Choose a desired arrow length (in meters).  Tweak as needed.
L = 100;  

% 3) If you want to thin out the arrows so the plot isn’t too dense,
%    pick a step size, e.g. plot one arrow every 5 points:
step = 1;
idxArrow = 1:step:size(traj,1);

% 4) Compute arrow‐components in 3D (U,V,W):
U =  L * cos( Psi(idxArrow) );   % x‐component
V =  L * sin( Psi(idxArrow) );   % y‐component
W =  zeros(size(U));      % no vertical component for a “heading” arrow

% 5) Actually draw them with quiver3.  The final “0” argument turns off auto‐scaling:
quiver3( X(idxArrow), Y(idxArrow), Z(idxArrow), ...
         U,             V,             W, ...
         0, ...                     % no automatic scaling
         'k', ...                   % arrow color (black)
         'LineWidth', 1, ...        % thickness of arrow lines
         'MaxHeadSize', 1 );        % size of the arrowheads

%% Smooth the trajectory

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

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(-115,60); axis equal
legend([hTraj, hStart, hReached, hGoal], ...
       {'Trajectory','Start','Reached z','Goal'}, ...
       'Location','best');
title(sprintf('FMT* smoothed solution (error to goal = %.2f m)', distToGoal));
