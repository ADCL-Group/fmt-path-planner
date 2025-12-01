function [trajSmooth, waypoints, speed] = smoothTrajectory(flightParams, bounds, planMap, rawTraj, wpts, goalRadius,anObst)
    % Smooth and interpolate a 4D UAV path using Dubins smoothing

    if nargin < 7 || isempty(anObst)
        anObst = [];   % default: no analytical obstacles
    end
    
    bounds(4,:) = [-pi, pi];

    conn = uavDubinsConnection( ...
            'MaxRollAngle',flightParams(1), ...
            'AirSpeed',flightParams(2), ...
            'FlightPathAngleLimit',flightParams(3:4));

    % Create the UAV state space
    ss = HelperUAVStateSpace( ...
        "MaxRollAngle",        flightParams(1), ...
        "AirSpeed",            flightParams(2), ...
        "FlightPathAngleLimit", [flightParams(3) flightParams(4)], ...
        "Bounds",              bounds         ...
    );

    % sv = validatorOccupancyMap3D(ss, "Map", planMap);
    % sv.ValidationDistance = 0.5;

    % Iterative Dubins‐based smoothing from start to end
    [shortPathFwd, isStuckFwd, interpFwd] = skipForward(ss, conn, planMap, rawTraj, wpts, goalRadius,anObst);

    if ~isStuckFwd
        shortPath = shortPathFwd;
        trajSmooth  = interpFwd(:,1:3);
    else
         % Iterative Dubins‐based smoothing from end to start
        [shortPathBack, isStuckBack, interpBack] = skipBack(ss, conn, planMap, rawTraj, wpts, goalRadius,anObst);
        if ~isStuckBack
            shortPath = shortPathBack;
            trajSmooth = interpBack(:,1:3);
        else
            % both methods failed
            warning( ...
              'Both forward‐skip and backward‐skip failed.');
            % pack rawTraj into a navPath
            shortPath = navPath(ss);
            append(shortPath, rawTraj);
            trajSmooth = rawTraj;
        end
    end
    
    waypoints = shortPath.States;
    speed = flightParams(2)*ones(size(trajSmooth,1),1);
end