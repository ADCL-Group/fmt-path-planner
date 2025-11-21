function [smoothWaypointsObj, isStuck, interpStatesAll] = skipBack(ss, conn, map, pthObj, targets, goalRadius, anObst)
% Skip redundant waypoints using backward‐search shortcut. Basically has a
% startNode that remains fixed and then loops backward (from the endNode) 
% to search the farthest end that can be connected. Once a valid jump is 
% found, the startNode is updated to the new waypoint and repeats the 
% process.

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

    startNode = 1;
    while startNode < M
        endNode = M;
        chosenInterp = [];
        found = false;

        while endNode > startNode+1
            % try connecting startNode to endNode
            [valid, ~, intposes] = isTrajValid(conn, map, waypts(startNode,:), waypts(endNode,:), anObst);
            
            if valid
                % we can go directly from startNode to endNode
                found = true;
                chosenInterp = intposes;
                break;
            end
            % that skip is not possible, reduce the skip
            endNode = endNode - 1;
        end

        if endNode == startNode+1 && ~found
            warning('Cannot connect waypoints. Aborting backward‐skip.');
            isStuck = true;
            break;
        end

        if firstSegment
            % Include the first point
            interpStatesAll = [interpStatesAll; chosenInterp];
            firstSegment = false;
        else
            % Skip the first state to avoid duplicating joint node
            interpStatesAll = [interpStatesAll; chosenInterp(2:end,:)];
        end
        
        isStateSkipped(endNode) = false; % keep that waypoint
        startNode = endNode; % move start forward
    end

    % wrap into a navPath to match MATLAB API
    smoothWaypointsObj = navPath(ss);
    % add smooth waypoints to the object.
    append(smoothWaypointsObj, waypts(~isStateSkipped, :));
end
