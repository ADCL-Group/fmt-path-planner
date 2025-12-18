function [path, goalFound, pathFound, S] = generatePath(S)
    if ~isfield(S,'generatedPathIdx') || isempty(S.generatedPathIdx)
        S.generatedPathIdx = [];
    end
    
    pathFound = false;

    % ---- Goal in tree? -> Backward path like C# ----
    [goalReached, idx] = isInTree(S.goalRegionIdx, S);
    if goalReached       % e.g., cost(goalIdx)<INF OR parent(goalIdx)~=0 (root allowed)
        S.generatedPathIdx = generatePathBackward(idx, S);
        goalFound = true;
        pathFound = true;
        path = S.V(S.generatedPathIdx, :);
        return
    end

    % ---- Otherwise do forward candidate from the root via "best child" ----
    [candidateIdx, ~, ~, S] = generatePathForward(S.rootIdx, S);
    newEnd = candidateIdx(end);

    if isempty(S.generatedPathIdx)
        S.generatedPathIdx = candidateIdx;
        pathFound = true;
    else
        % Replace only if the new terminal is closer to the goal region
        oldEnd = S.generatedPathIdx(end);
        if endToGoalRegionDist(newEnd, S) < endToGoalRegionDist(oldEnd, S)
            S.generatedPathIdx = candidateIdx;
            pathFound = true;
        end
    end

    % Finish flags (mirror C#): only "finished/success" when full path exists
    goalFound = false;
    path = S.V(S.generatedPathIdx, :);
end

    