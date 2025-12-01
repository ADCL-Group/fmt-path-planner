function [S, newlyBlocked, newlyUnblocked] = updateObstructedNodes(dynObs, S)
% Update block state using dynamic obstacle
% - S.blocked : tree state (blocked due to cost==inf and propagation)
% - S.dynamicObstructed : nodes currently inside a dynamic obstacle zone
% - recalcChildrenCost : used only to recompute real edge costs for descendants

    % Build current dynamic obstruction mask (obstacle-zone only)
    N = size(S.V,1);
    cur = false(N,1);

    if ~isfield(S,'dynObs') 
        S.dynObs = dynObs;
    end

    if ~isempty(dynObs)
        r2 = S.safeRadiusDObstacle^2;
        D2 = pdist2(S.V(:,1:3), dynObs(:,1:3), 'squaredeuclidean');
        cur = any(D2 < r2, 2);
    end

    S.dynamicObstructed = cur;

    % Compute transitions relative to previous dynamic mask
    if ~isfield(S,'dynamicObstructedPrev') || numel(S.dynamicObstructedPrev) ~= N
        S.dynamicObstructedPrev = false(N,1);
    end
    newlyBlocked   =  cur & ~S.dynamicObstructedPrev;
    newlyUnblocked = ~cur &  S.dynamicObstructedPrev;
    S.dynamicObstructedPrev = cur;

    % Newly blocked nodes -> cost = inf, block descendants
    bIdx = find(newlyBlocked);
    for t = bIdx(:).'
        if ~S.blocked(t)
            S.cost(t)    = inf;
            S.blocked(t) = true;
            S = recalcChildrenCost(t, S);
        end
    end

    % Newly unblocked nodes -> recompute cost
    uIdx = find(newlyUnblocked);
    for t = uIdx(:).'
        p = S.parent(t);
        if p == 0                          % root
            S.cost(t) = 0;
            S.blocked(t) = false;
            S = recalcChildrenCost(t, S);  % will also unblock descendants
        elseif S.cost(p) < inf             % parent is reachable
            % Parent is reachable: compute edge cost p -> t with Dubins (or line)
            parentState = S.V(p,:);
            childState  = S.V(t,:);

            if ~isempty(S.conn)
                % Dubins case
                aux1 = [0 0 0 parentState(4)];
                aux2 = [0 0 0 childState(4)];
                headingPenalty = costMetric(aux1, aux2, S.w);

                [~, edgeLen, ~] = interpDubins(S.conn, parentState, childState, 0);
                S.cost(t) = S.cost(p) + edgeLen + headingPenalty;
            else
                % Straight-line
                res = S.map.Resolution;
                [~, edgeLen, ~] = interpLine(res, parentState, childState, 0);
                S.cost(t) = S.cost(p) + edgeLen;
            end

            S.blocked(t) = false;

            % Now recompute all descendants of t with true edge cost
            S = recalcChildrenCost(t, S);
        else
            % parent still blocked -> leave as is (still blocked)
        end
    end
end