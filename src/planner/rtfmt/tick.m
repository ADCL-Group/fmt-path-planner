function S = tick(dynObs, currPose, S)
    S = updateObstructedNodes(dynObs, S); % block/unblock near dynamic obstacles

    if nargin > 1 && ~isempty(currPose)
        S = setCurrentPose(currPose, S);
    end

    nSteps = S.expandTreeRate;
    for ii = 1:nSteps
        S = rewireLocally(S);   % local rewire of newly unblocked children
        S = expandTree(S);      % lazy neighbor, argmin parent, add child
        S = rewireFromRoot2(S); % breadth rewire around root (variant 2)
    end

    childIdx = find(S.parent > 0); % Find all vertex xi that got a parent
    E = [S.parent(childIdx), childIdx];
    S.E = E;
end

    