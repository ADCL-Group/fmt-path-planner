function [yMin, yCost, isCFixed] = selectBestParent(xIdx, YNear, S)
% xIdx: index of child node
% YNear: indices of candidate parents (Open)
% S: struct with V, cost, map, conn, etc.

    yMin = [];
    yCost = inf;
    isCFixed = false;

    if isempty(YNear)
        return;
    end

    % States
    parentsStates = S.V(YNear,:);          % [nParents x stateDim]
    childState = S.V(xIdx,:);           % [1 x stateDim]
    costs_y = S.cost(YNear);         % parent costs

    if ~isempty(S.conn)
        % Dubins case
        aux1 = [zeros(size(S.V(YNear,1:3))) S.V(YNear,4)];
        aux2 = [zeros(size(S.V(xIdx,1:3))) S.V(xIdx,4)];
        headingPenalty = costMetric(aux1, aux2, S.w);

        [pts, totalCost, bestLocalIdx] = interpDubins(S.conn, parentsStates, childState, costs_y + headingPenalty);
    else
        % Straight-line
        res = S.map.Resolution;

        [pts, totalCost, bestLocalIdx] = interpLine(res, parentsStates, childState, costs_y);
    end
    

    % Map bestLocalIdx back to global index in V
    yMin  = YNear(bestLocalIdx);
    yCost = totalCost;   % already cost(parent) + path length

    % 2) STATIC obstacle check using pts
    occ = checkOccupancy(S.map, pts(:,1:3));
    tf_static = all(occ < S.map.FreeThreshold);
    if ~tf_static
        isCFixed = false;
        return;
    end

    % 3) DYNAMIC obstacle checks, reusing your current logic
    if ~isfield(S,'dynObs') || isempty(S.dynObs)
        isCFixed = true;
        return;
    end

    % If either endpoint already dynamically obstructed, block
    if S.dynamicObstructed(yMin) || S.dynamicObstructed(xIdx)
        isCFixed = false;
        return;
    end

    if segmentIntersectsSphere3D(pts, S.dynObs, S.safeRadiusDObstacle)
        isCFixed = false; return;
    end

    % pi = S.V(yMin,1:3);
    % pj = S.V(xIdx,1:3);
    % 
    % for k = 1:size(S.dynObs,1)
    %     c = S.dynObs(k,1:3);
    %     R = S.safeRadiusDObstacle;
    % 
    %     if segmentIntersectsSphere3D(pi, pj, c, R)
    %         isCFixed = false;
    %         return;
    %     end
    % end

    isCFixed = true;
end