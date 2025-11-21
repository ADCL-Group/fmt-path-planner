function [isValid, dist, interp] = isTrajValid(conn, map, state1, state2, anObst)

    if nargin < 5 || isempty(anObst)
        anObst = [];   % default: no analytical obstacles
    end

    % Create a Dubins path
    uavDubPathSeg=connect(conn,state1,state2);
    dist = uavDubPathSeg{1}.Length;

    if isnan(dist)
        % in case the distance between 2 states is NaN then set
        % motion invalid
        isValid = false;
        return
    end

    % interval = 50/dist;
    % alpha = [0:interval:1 1];

    % sampleDists=alpha*dist;
    maxStep = 10;   % max spacing between samples [meters]
    nSamples = max( ceil(dist/maxStep) + 1, 3 );  % at least 3 points

    sampleDists = linspace(0, dist, nSamples);
    [intposes]=interpolate(uavDubPathSeg{1}, sampleDists);

    % Analytic obstacles first (fast reject)
    if ~isempty(anObst)
        insideAna = isInsideObstacles(intposes(:,1:3), anObst);  % N x 1 logical
        if any(insideAna)
            % Trajectory intersects analytic obstacle → invalid
            isValid = false;
            dist    = Inf;
            return;
        end
    end

    % Check all interpolated states for validity
    % MATLAB checks by doing ~isOccupied which is more restrictive because
    % unknown spaces (-1) in the map are marked as occupied. Checking if
    % the value is less than the map's threshold sets the unknown as free.
    isOccupied = map.checkOccupancy(intposes(:,1:3));
    % interpValid = ~isOccupied;
    occValid = isOccupied < map.FreeThreshold;

    interpValid = occValid;% & cylValid;

    % Find the first invalid index. Note that the maximum non-empty
    % value of firstInvalidIdx can be 2, since we always check
    % state1 and state2 and state1 is already verified above.
    firstInvalidIdx = find(~interpValid, 1);

    if isempty(firstInvalidIdx)
        % The whole motion is valid
        isValid = true;
        interp = intposes;
    else
        isValid = false;
        dist = Inf;
        interp = [];
    end
end