function [freeIdx, freeStates] = isStateValid(map, state)
    % Check all states for validity
    % MATLAB checks by doing ~occ which is more restrictive because
    % unknown spaces (-1) in the map are marked as occupied. Checking if
    % the value is less than the map's threshold sets the unknown as free.
    occ = checkOccupancy(map, state(:,1:3));
    freeIdx = occ < map.FreeThreshold;
    freeStates = state(freeIdx, :);
end

