function [yMin, yMinCost] = argMinCost(x, YNear, checkFixed, S)
    yMin = [];
    yMinCost = Inf;

    if isempty(YNear)
        return;
    end

    YNear = YNear(:);

    pathCosts = costMetric(S.V(YNear,:), S.V(x,:), S.w);
    pathCosts = pathCosts(:);

    costs_y = S.cost(YNear);
    costs_y = costs_y(:);

    % Total cost for each candidate y
    totalCosts = costs_y + pathCosts;

    if checkFixed
        for k = 1:numel(YNear)
            if ~isEdgeFixedFree(YNear(k), x, S)
                totalCosts(k) = Inf;  % invalidate this candidate
            end
        end
    end

    [yMinCost, idxMin] = min(totalCosts);

    if isfinite(yMinCost)
        yMin = YNear(idxMin);
    else
        yMin = [];
        yMinCost = Inf;
    end
end

    