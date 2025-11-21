function [traj, Ecell, Vcell] = FMTWaypoints(map3D, limits, wpts, N, rn, goalRadius, w, flightParams, anObst)
    % Run FMT* over a sequence of waypoints

    if nargin < 9 || isempty(anObst)
        anObst = [];           % default: no analytic obstacles
    end
    if nargin < 8 || isempty(flightParams)
        flightParams = [];     % default: no Dubins, straight-line
    end
    if nargin < 7 || isempty(w)
        w = 0;                 % default: zero weight
    end

    d = size(limits,1);
    M = size(wpts,1);
    numSeg = M - 1;

    trajAll = [];
    Ecell = cell(numSeg,1);
    Vcell = cell(numSeg,1);

    if isscalar(goalRadius)
        goalR = repmat(goalRadius, numSeg, 1);
    else
        goalR = goalRadius;
    end

    start = wpts(1,:);
    for i = 1:numSeg
        goal  = wpts(i+1,:);
        try
            [traj, E, V] = FMT(map3D, limits, start, goal, N, rn, goalR(i), w, flightParams, anObst);
        catch ME
            fprintf("Cannot connect waypoint %d to %d\n",i,i+1);
            disp(ME.message)
            traj = wpts(i+1,:);
            E = [];
            V = [];
        end
        Ecell{i} = E;
        Vcell{i} = V;

        if i==1
            % first segment: take entire traj
            trajAll = traj;
        else
            % subsequent: skip duplicate start point
            trajAll = [trajAll; traj(2:end,:)]; %#ok<AGROW>
        end

        % Start next segment from the endpoint of the previous one
        start = traj(end, :);
    end
    
    if d == 3
        psi = computeHeading(trajAll);
        traj = [trajAll, psi];
    elseif d == 4
        traj = trajAll;
    end
end
