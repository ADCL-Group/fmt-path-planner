function [candidatePath, hitGoal, leafNode, S] = generatePathForward(root, S)
    % Build a forward path from root using "best child" selection.

    N = numel(S.parent);

    % Precompute goal mask
    goalMask = false(N,1);
    goalMask(S.goalRegionIdx) = true;

    candidatePath = zeros(N,1);
    k = 1;
    candidatePath(1) = root;

    c = root;
    hitGoal = goalMask(c);
    leafNode = [];

    while ~hitGoal
        y = findBestChild(c, S);
        if ~isempty(y)
            c = y;
            k = k + 1;
            candidatePath(k) = c;
            hitGoal = goalMask(c);
        else
            % leaf (no children of c)
            leafNode = c;
            S.checkedPath(c) = true;
            S.checkedPathCandidates(end + 1) = c;
            break;
        end
    end

    candidatePath = candidatePath(1:k);

end