function [smoothWaypointsObj, isStuck, interpStatesAll] = skipForward(ss,conn, map, pthObj,targets,goalRadius, anObst)
% Based on exampleHelperUavPathSmoothing from the Motion Planning with RRT 
% for Fixed-Wing UAV Example from MATLAB.
% It has a startNode that remains fixed and then loops forward to search
% the farthest end that can be connected. Once a valid jump is found, the
% startNode is updated to the new waypoint and repeats the process.

    if nargin < 7 || isempty(anObst)
        anObst = [];   % default: no analytical obstacles
    end
    
    if isa(pthObj,'navPath')
        waypts = pthObj.States;
    else
        waypts = pthObj;
    end
    
    M = size(waypts,1);
    isStuck = false;

    if M < 3
        smoothWaypointsObj = navPath(ss);
        append(smoothWaypointsObj, waypts);
        interpStatesAll = waypts;
        return
    end

    validStates = isStateValid( map, waypts(:,1:4), anObst );
    if any(~validStates)
        % If any single state is invalid, we cannot shorten
        warning('One or more input waypoints are invalid.');
        smoothWaypointsObj = navPath(ss);
        append(smoothWaypointsObj, waypts);
        interpStatesAll = waypts;
        return
    end
    
    % Initialize the skipping mask: true = “skip”, false = “keep”
    isStateSkipped = true(M,1);

    % Set the targets as false to keep them
    K = size(targets,1);
    if isscalar(goalRadius)
        goalR = repmat(goalRadius, K, 1);
    elseif numel(goalRadius) == K-1
        goalR = goalRadius(:);
    end

    isStateSkipped(1) = false;
    for i = 2:K
        % Compute Euclidean distances from all waypoints to target
        diffXYZ = waypts(:,1:3) - targets(i,1:3);  % M×3
        dists   = sqrt( sum(diffXYZ.^2, 2) );      % M×1

        % Find indices where distance <= radii(i)
        idxNear = (dists <= goalR(i));
        if ~isempty(idxNear)
            % Pick the one with minimum distance
            [~, idxMin] = min(dists);
            isStateSkipped(idxMin) = false;
        end
    end

    % isStateSkipped([1, end]) = false;

    interpStatesAll = [];
    firstSegment    = true;

    lastValidInterp   = [];

    startNode = 1;
    endNode = startNode + 1;
    while(endNode <= M)
        % try connecting startNode to endNode
        [valid, ~, intposes] = isTrajValid(conn, map, waypts(startNode,:), waypts(endNode,:),anObst);

        if valid
            % we can go directly from startNode to endNode
            lastValidInterp  = intposes;
            endNode = endNode + 1; % try to skip even further
        else
            % that skip is not possible
            if (endNode - startNode) == 1
                % Two adjacent waypoints cannot be directly connected. Stuck.
                warning('Cannot connect waypoints. Aborting forward‐skip.');
                isStuck = true;
                break;
            end
            isStateSkipped(endNode-1) = false;
            startNode = endNode - 1; % reset the start to last good node

            % We know the last valid jump is startNode -> lastValidEndNode.
            % Append its interpolation to the full trajectory.
            if ~isempty(lastValidInterp)
                if firstSegment
                    % Include first point only once
                    interpStatesAll = [interpStatesAll; lastValidInterp];
                    firstSegment = false;
                else
                    % Skip the first state to avoid duplicating the junction
                    interpStatesAll = [interpStatesAll; lastValidInterp(2:end,:)];
                end
            end

        end
    end

    % If we exited because endNode > M, we may still have a pending valid jump
    if ~isStuck && ~isempty(lastValidInterp)
        if firstSegment
            interpStatesAll = [interpStatesAll; lastValidInterp];
        else
            interpStatesAll = [interpStatesAll; lastValidInterp(2:end,:)];
        end
    end

    if isempty(interpStatesAll)
        interpStatesAll = waypts;
    end
    
    % wrap into a navPath to match MATLAB API
    smoothWaypointsObj = navPath(ss);
    % add smooth waypoints to the object.
    append(smoothWaypointsObj, waypts(~isStateSkipped, :));

end

