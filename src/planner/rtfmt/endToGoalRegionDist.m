function d = endToGoalRegionDist(endIdx, S)
% Euclidean distance from node 'endIdx' to the nearest node in 'goalSet'
    diffs = S.V(S.goalRegionIdx,:) - S.V(endIdx,:);
    d = min(vecnorm(diffs, 2, 2));
end

    