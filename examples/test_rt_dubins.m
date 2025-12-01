clc; close all; clearvars;
rng(1,"twister");
setupPath;

result = buildOccupancyMap("data/osm/boston.osm", 0.1, 20, 35);
map = result.maps{3};
mapLimits = result.mapLimits;

limits = [mapLimits; 0 2*pi];

start = [ 828 7 155 ];
wpt1  = [ 694 200 170 ];
wpt2  = [ 453 595 205 ];
wpt3  = [ 100 641 222 ];
wpt4  = [ -290 590 225 ];
wpt5  = [ -440 250 200 ];
goal = [ -600 -600 120 ];

kts2ms = 1/1.94384001;
MaxRollAngle         = 35*pi/180;
AirSpeed             = 100*kts2ms;
FlightPathAngleLimit = [-10, 10]*pi/180;

flightParams = [MaxRollAngle, AirSpeed, FlightPathAngleLimit];

g = 9.81;
w = AirSpeed^2/(g*tan(MaxRollAngle));

wpts = [start; wpt1; wpt2; wpt3; wpt4; wpt5; goal];
wpts(:,4) = computeHeading(wpts);

N = 100;

rn = optimalRadius(N+2, limits, w); % radius to find neighbors
goalRadius = rn/2; % radius of the goal area around the goal wpt

%%
close all; clc

rng(1,"twister");
planner = rt_fmt_planner(map, limits(1:4,:), wpts(1,:), wpts(end,:), rn, ...
    'flightParams', flightParams, ...
    'N', N, ...
    'w', w, ...
    'expandTreeRate', 64, ...
    'safeRadiusDObstacle', 100, ...
    'goalRadius', goalRadius);

figure; show(map); 
hold on; grid on; axis equal
hTree = plot3(NaN,NaN,NaN,'r-','LineWidth',1);    % current planned (spliced) path
hTrail= plot3(NaN,NaN,NaN,'g-','LineWidth',2);    % committed trail
hPos  = plot3(NaN,NaN,NaN,'ko','MarkerFaceColor','y','MarkerSize',6);
hDubins  = plot3(NaN,NaN,NaN,'r-','LineWidth',2);    % Dubins trail
dubinsTrail = zeros(0,3);

% simulation/stepper state
q_now = wpts(1,:); % current pose      
trail = q_now; % committed path
tree = []; % proposed local path

maxIters = 1000;

dynamicObstacles = [];

currentPath = [];   % last path returned by planner
pathIdx     = 1;    % index of the waypoint we are currently at in currentPath

for it = 1:maxIters
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

            diffs = path(:,1:3) - q_now(1:3);           % Nx3
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
            oldIdx = pathIdx;               % previous waypoint index
            pathIdx = pathIdx + 1;          % advance one node

            q_prev = currentPath(oldIdx,:);     % previous robot pose
            q_now  = currentPath(pathIdx,:);    % new robot pose

            trail = [trail; q_now];
            dubSeg = connect(planner.conn, q_prev, q_now);

            if ~isempty(dubSeg) && ~isnan(dubSeg{1}.Length)
                dist = dubSeg{1}.Length;
                s = linspace(0, dist, 50);
                states = interpolate(dubSeg{1}, s);

                % accumulate sampled Dubins points
                dubinsTrail = [dubinsTrail; states(:,1:3)];
            end

        else
            q_now = currentPath(pathIdx,:);
        end
    end

    % update visuals
    set(hTree,  'XData', tree(:,1), 'YData', tree(:,2), 'ZData', tree(:,3));
    if ~isempty(dubinsTrail)
        set(hDubins, 'XData', dubinsTrail(:,1), ...
                     'YData', dubinsTrail(:,2), ...
                     'ZData', dubinsTrail(:,3));
    end

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

E = planner.E; V = planner.V;

if size(E,1) <= 5000
    figure;
    show(map); alpha(0.3);
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
         'Color',    [1 0 0], ...  % light gray
         'LineWidth', 0.5);

    plot3(trail(:,1),trail(:,2),trail(:,3),'g-','LineWidth',2);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal
end