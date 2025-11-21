function [freeIdx, freeStates] = isStateValid(map, state, anObst)
    % Check all states for validity
    % MATLAB checks by doing ~occ which is more restrictive because
    % unknown spaces (-1) in the map are marked as occupied. Checking if
    % the value is less than the map's threshold sets the unknown as free.

    if nargin < 3 || isempty(anObst)
        anObst = [];   % default: no analytical obstacles
    end
    
    N = size(state, 1);
    insideAna = false(N,1);

    if ~isempty(anObst)
        insideAna = isInsideObstacles(state(:,1:3), anObst);  % N x 1 logical
    end

    idxToCheck = find(~insideAna);

    occFree = false(N, 1);
    if ~isempty(idxToCheck)
        % MATLAB checks by doing ~occ which is more restrictive because
        % unknown spaces (-1) in the map are marked as occupied. Checking if
        % the value is less than the map's threshold sets the unknown as free.
        occ = checkOccupancy(map, state(idxToCheck,1:3));
        occFree(idxToCheck) = (occ < map.FreeThreshold);
    end

    freeIdx = (~insideAna) & occFree;
    freeStates = state(freeIdx, :);
end

