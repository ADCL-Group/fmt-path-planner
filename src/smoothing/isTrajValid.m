function [isValid, dist] = isTrajValid(conn, map, state1, state2)
    % Create a Dubins path
    uavDubPathSeg=connect(conn,state1,state2);
    dist = uavDubPathSeg{1}.Length;

    if isnan(dist)
        % in case the distance between 2 states is NaN then set
        % motion invalid
        isValid = false;
        return
    end

    interval = 50/dist;
    alpha = [0:interval:1 1];

    sampleDists=alpha*dist;
    [intposes]=interpolate(uavDubPathSeg{1}, sampleDists);

    % Check all interpolated states for validity
    % MATLAB checks by doing ~isOccupied which is more restrictive because
    % unknown spaces (-1) in the map are marked as occupied. Checking if
    % the value is less than the map's threshold sets the unknown as free.
    isOccupied = map.checkOccupancy(intposes(:,1:3));
    % interpValid = ~isOccupied;
    interpValid = isOccupied < map.FreeThreshold;

    % Find the first invalid index. Note that the maximum non-empty
    % value of firstInvalidIdx can be 2, since we always check
    % state1 and state2 and state1 is already verified above.
    firstInvalidIdx = find(~interpValid, 1);

    if isempty(firstInvalidIdx)
        % The whole motion is valid
        isValid = true;
    else
        isValid = false;
        dist = Inf;
    end
end