function S = tick(dynObs, currPose, S)
    % fprintf("\tinside1:%d",length(find(S.blocked)));
    S = updateObstructedNodes(dynObs, S); % block/unblock near dynamic obstacles
    % fprintf("\tinside2:%d",length(find(S.blocked)));

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
    % generatePath (if goal already in tree do backward, else forward)            
    % N.B. we just expose backtrackPath(); caller can check hasPath()
end

    