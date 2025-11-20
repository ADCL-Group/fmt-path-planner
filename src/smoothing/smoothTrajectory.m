function [trajSmooth, speed] = smoothTrajectory(flightParams, bounds, planMap, rawTraj, wpts, goalRadius)
    % Smooth and interpolate a 4D UAV path using Dubins smoothing
    
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
    [shortPathFwd, isStuckFwd] = skipForward(ss, conn, planMap, rawTraj, wpts, goalRadius);

    if ~isStuckFwd
        shortPath = shortPathFwd;
    else
         % Iterative Dubins‐based smoothing from end to start
        [shortPathBack, isStuckBack] = skipBack(ss, conn, planMap, rawTraj, wpts, goalRadius);
        if ~isStuckBack
            shortPath = shortPathBack;
        else
            % both methods failed
            warning( ...
              'Both forward‐skip and backward‐skip failed.');
            % pack rawTraj into a navPath
            shortPath = navPath(ss);
            append(shortPath, wpts);
        end
    end

    % Determine if there are loops
    [trajSmooth, speed] = smartInterpolation(flightParams, shortPath.States, ss, 2000);
end

function [interpStates, speed] = smartInterpolation(flightParams, wpts, ss, numStates)
    mxRoll = flightParams(1);
    arspeed = flightParams(2);
    flightpathangle = flightParams(3:4);

    connFast = uavDubinsConnection('MaxRollAngle',mxRoll,'AirSpeed',arspeed,'FlightPathAngleLimit',flightpathangle);
    connSlow = uavDubinsConnection('MaxRollAngle',mxRoll,'AirSpeed',arspeed,'FlightPathAngleLimit',flightpathangle);
    
    % Retrieve lengths of path segments
    pLengths = ss.distance(wpts(1:end-1,:), wpts(2:end,:));
    ptsToAdd = numStates-size(wpts,1);
    newNumStates = numStates;

    % Get baseline number of pts needed to be added to each segment
    ptsToAddPerSegment = floor(pLengths/sum(pLengths)*ptsToAdd);
    ptsToAddPerSegment(isnan(ptsToAddPerSegment)) = 0;

    % Update remaining points
    ptsToAdd = ptsToAdd - sum(ptsToAddPerSegment);

    % Update the point-to-point distance for each segment using the
    % new distribution of points. For example, if segment 1
    % initially had length 2, and was assigned 2 points, its new
    % point-to-point distance is 2/(3+1) = 1/2
    pLengths = pLengths./(ptsToAddPerSegment+1);

    % Distribute the K remaining points based on the segments with
    % K-largest point-to-point distances
    [~, idxs] = maxk(pLengths, ptsToAdd);
    ptsToAddPerSegment(idxs) = ptsToAddPerSegment(idxs) + 1;

    deltaSpeed = 0; 
    while deltaSpeed <= 5

        deltaSpeed = deltaSpeed + 1;
        connSlow.AirSpeed = arspeed - deltaSpeed;

        % Pre-allocate new states
        interpStates = zeros(newNumStates, ss.NumStateVariables);
    
        speed = zeros(newNumStates,1);
    
        % Loop through segments and add points where needed
        curPt = 1;
        for i = 1:numel(pLengths)
            if ptsToAddPerSegment(i) > 0
                % Interpolate extra points
                interval = linspace(0,1,ptsToAddPerSegment(i)+2);
                fraction = interval(1:end-1);
    
                % Select which path is shorter (no loop)
                fastPaths = connect(connFast, wpts(i,:), wpts(i+1,:));
                fastOK  = find(~arrayfun(@(p)isDetour(p,0.30), fastPaths),1);
                
                slowPaths = connect(connSlow, wpts(i,:), wpts(i+1,:));
                slowOK  = find(~arrayfun(@(p)isDetour(p,0.30), slowPaths),1);
        
                if isempty(fastOK) && ~isempty(slowOK)
                    chosen = slowPaths;   % fast loops, slow clean
                    speed(curPt:curPt+ptsToAddPerSegment(i)) = arspeed - deltaSpeed;
                elseif ~isempty(fastOK)
                    chosen = fastPaths;   % fast already clean
                    speed(curPt:curPt+ptsToAddPerSegment(i)) = arspeed;
                else
                    chosen = slowPaths;   % both loop, keep slow
                    speed(curPt:curPt+ptsToAddPerSegment(i)) = arspeed - deltaSpeed;
                end
        
                lengths=fraction*chosen{1}.Length;
                [intposes]=interpolate(chosen{1}, lengths);
                
                % Return the [x,y,z,psi] from the interpolated states
                interpState=intposes(:,1:4);
                
    
                interpStates(curPt:curPt+ptsToAddPerSegment(i),:) = interpState;
                curPt = curPt + ptsToAddPerSegment(i) + 1;
            else
                % Just add starting pt in segment
                interpStates(curPt,:) = wpts(i,:);
                curPt = curPt + 1;
            end
        end

        if isempty(find(speed < 0,1))
            break;
        end
    end

    % Add the last state
    interpStates(end,:) = wpts(end,:);
    speed(end) = speed(end-1);
    
end

function tfLoop = isDetour(pathObj, detourThresh)
    % isDetour  True if Dubins path is a big detour
    
    L = pathObj{1}.Length;
    if isnan(L)
        tfLoop = true;
        return;
    end
    
    chord = norm(pathObj{1}.GoalPose(end,1:3) - pathObj{1}.StartPose(1,1:3));
    
    rho = (L - chord) / chord;
    tfLoop = rho >= detourThresh;
end