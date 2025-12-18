clc; close all; clearvars;
rng(1,"twister");
setupPath;

result = buildOccupancyMap("data/osm/boston.osm", 0.1, 20, 35);
origMap = result.maps{1};
planMap = result.maps{3};
mapLimits = result.mapLimits;

limits = [mapLimits; 0 2*pi];

start = [ 828 7 155 ];
goal = [ -600 -600 120 ];

N = 5000;

rn = optimalRadius(N+2, limits(1:3,:)); % radius to find neighbors
goalRadius = rn/2; % radius of the goal area around the goal wpt

%%
close all; clc

rng(1,"twister");
planner = rt_fmt_planner(planMap, limits(1:3,:), start, goal, rn, ...
    'N', N, ...
    'w', 0, ...
    'expandTreeRate', 32, ...
    'safeRadiusDObstacle', 100, ...
    'goalRadius', goalRadius);

figure; show(origMap); 
hold on; grid on; axis equal
hTree = plot3(NaN,NaN,NaN,'r-','LineWidth',1);    % current planned (spliced) path
hTrail= plot3(NaN,NaN,NaN,'g-','LineWidth',2);    % committed trail
hPos  = plot3(NaN,NaN,NaN,'ko','MarkerFaceColor','y','MarkerSize',6);

% simulation/stepper state
q_now = start; % current pose      
trail = q_now; % committed path
tree = []; % proposed local path

maxIters = 1000;

% Dynamic obstacles [x,y,z]
dynamicObstacles = [-260, -490, 150;  % obstacle 1
                    470 0 150;  % obstacle 2
                    110 300 150];  % obstacle 3
% dynamicObstacles = [];

% [vx,vy,vz]
speed = [ 0   0  15; % obstacle 1
          10 0 0; % obstacle 2
          0 0 -15]; % obstacle 3
dir = sign(speed);

safeR = isfield(planner, 'safeRadiusDObstacle') * planner.safeRadiusDObstacle ...
      + ~isfield(planner, 'safeRadiusDObstacle') * planner.rn;

hObs   = gobjects(size(dynamicObstacles,1),1);
hBubble= gobjects(size(dynamicObstacles,1),1);

for k = 1:size(dynamicObstacles,1)
    c = dynamicObstacles(k,:);
    
    hObs(k) = plot3(NaN, NaN, NaN, 'ro', 'MarkerFaceColor','r', 'MarkerSize',6);
    
    hBubble(k) = drawOrUpdateSphere([], c, safeR);
end

currentPath = [];   % last path returned by planner
pathIdx     = 1;    % index of the waypoint we are currently at in currentPath

for it = 1:maxIters

    % Motion of the dynamic obstacles
    for k = 1:size(dynamicObstacles,1)
        p  = dynamicObstacles(k,:);
        v  = speed(k,:);
        s  = abs(v);
        d  = dir(k,:);

        % advance
        p = p + d .* s;

        % bounce logic for each axis independently
        [p(1), d(1)] = bounce1D(p(1), d(1), s(1), limits(1,1), limits(1,2));
        [p(2), d(2)] = bounce1D(p(2), d(2), s(2), limits(2,1), limits(2,2));
        [p(3), d(3)] = bounce1D(p(3), d(3), s(3), limits(3,1), limits(3,2));

        dynamicObstacles(k,:) = p;
        dir(k,:)              = d;

        % update visuals
        set(hObs(k), 'XData', p(1), 'YData', p(2), 'ZData', p(3));
        hBubble(k) = drawOrUpdateSphere(hBubble(k), p, safeR);
    end

    planner = tick(dynamicObstacles, q_now, planner);

    [path, goalHit, pathFound, planner] = generatePath(planner);
    if isempty(path) || size(path,1) < 2
        % nothing to follow yet; keep expanding
        if ismember(planner.rootIdx, planner.goalRegionIdx)
            break;
        end

        continue
    end

    if pathFound || isempty(currentPath)
        % We got a *better* path (or first ever path),
        % so we want to switch to it.
        % New better path from planner: path is from *current root* q_now.
        % We want continuity: find point on new path closest to q_now
        % and start from there.

        if ~isempty(path) && size(path,1) >= 2

            diffs = path - q_now;           % Nx3
            d2    = sum(diffs.^2, 2);       % squared distances
            [~, k] = min(d2);               % index of closest waypoint
    
            currentPath = path;
            pathIdx     = k;                % we'll advance from here
    
            % Update global "tree" without duplicating the junction point
            if isempty(tree)
                tree = currentPath;
            else
                if isequal(currentPath(1,:), tree(end,:))
                    tree = [tree; currentPath(2:end,:)];
                else
                    tree = [tree; currentPath];
                end
            end
        end
    end

    % move one step along currentPath
    if ~isempty(currentPath)
        if pathIdx < size(currentPath,1)
            pathIdx = pathIdx + 1;          % advance one node
        end
        q_now = currentPath(pathIdx,:);     % new robot pose
        trail = [trail; q_now];
    end

    % update visuals
    set(hTree,  'XData', tree(:,1), 'YData', tree(:,2), 'ZData', tree(:,3));

    set(hTrail, 'XData', trail(:,1), 'YData', trail(:,2), 'ZData', trail(:,3));
    set(hPos,   'XData', q_now(1),  'YData', q_now(2),  'ZData', q_now(3));

    pause(0.01)
    drawnow;

    if ismember(planner.rootIdx, planner.goalRegionIdx)
        break; % Finished following the path!
    end

    if goalHit
        fprintf('Goal reached in %d iterations, stopping loop\n',it);
    end
end

% Plot the FMT tree
E = planner.E; V = planner.V;

if size(E,1) <= 5000
    figure;
    show(planMap); alpha(0.3);
    hold on;
    
    % Extract endpoints
    p1 = V(E(:,1), :);   % parents
    p2 = V(E(:,2), :);   % childs
    
    % Build 2×P arrays for line()
    X = [p1(:,1)'; p2(:,1)'];
    Y = [p1(:,2)'; p2(:,2)'];
    Z = [p1(:,3)'; p2(:,3)'];
    
    % Draw all segments
    line(X, Y, Z, ...
         'Color',    [1 0 0], ...
         'LineWidth', 0.5);

    plot3(trail(:,1),trail(:,2),trail(:,3),'g-','LineWidth',2);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal
end


function h = drawOrUpdateSphere(h, center, radius)
% Draws (or updates) a sphere centered at 'center' with 'radius'.
% If h is empty, creates a new surf; otherwise updates X/Y/Z of existing one.

    n = 24;  % mesh density
    [X,Y,Z] = sphere(n);
    X = center(1) + radius * X;
    Y = center(2) + radius * Y;
    Z = center(3) + radius * Z;

    if isempty(h) || ~isvalid(h)
        h = surf(X, Y, Z, ...
            'EdgeColor', 'none');
    else
        set(h, 'XData', X, 'YData', Y, 'ZData', Z);
    end
end

function [x, d] = bounce1D(x, d, stepMag, xmin, xmax)
% Bounce a 1D coordinate x inside [xmin,xmax] with direction d in {-1,+1}.
    if stepMag==0, return; end
    if x >= xmax
        x = xmax; d = -1;
    elseif x <= xmin
        x = xmin; d = +1;
    end
end
