function [S, newlyBlocked, newlyUnblocked] = updateObstructedNodes(dynObs, S)
% Update block state using dynamic obstacle *transitions* like the C# code.
% - S.blocked : tree state (blocked due to cost==inf and propagation)
% - S.dynamicObstructed : nodes currently inside a dynamic obstacle zone
% - recalcChildrenCost_fromChildren : must *not* overwrite the seed’s cost

    %----- 1) Build current dynamic obstruction mask (obstacle-zone only) -----
    N = size(S.V,1);
    cur = false(N,1);

    if ~isfield(S,'dynObs') 
        S.dynObs = dynObs;
    end

    if ~isempty(dynObs)
        r2 = S.safeRadiusDObstacle^2;
        % vectorized distance check
        % (avoid loops; for big trees use chunking if memory tight)
        D2 = pdist2(S.V(:,1:3), dynObs(:,1:3), 'squaredeuclidean');
        cur = any(D2 < r2, 2);
    end

    % Preserve for rewireLocally’s “isInDynamicObstacle”
    S.dynamicObstructed = cur;

    %----- 2) Compute transitions relative to previous dynamic mask -----
    if ~isfield(S,'dynamicObstructedPrev') || numel(S.dynamicObstructedPrev) ~= N
        S.dynamicObstructedPrev = false(N,1);
    end
    newlyBlocked   =  cur & ~S.dynamicObstructedPrev;
    newlyUnblocked = ~cur &  S.dynamicObstructedPrev;
    S.dynamicObstructedPrev = cur;   % advance the “sensor” state

    % NOTE: Do *not* set S.blocked = cur.
    % S.blocked is the propagated tree state maintained via cost INF and recalc.

    %----- 3) Apply C#-style blocking on newly blocked nodes -----
    bIdx = find(newlyBlocked);
    for t = bIdx(:).'
        if ~S.blocked(t)       % mimic “if (!blockedNodes.Contains(node))”
            S.cost(t)    = inf;
            S.blocked(t) = true;
            S = recalcChildrenCost(t, S);  % will also block descendants
        end
    end

    %----- 4) Apply C#-style unblocking on newly unblocked nodes -----
    uIdx = find(newlyUnblocked);
    for t = uIdx(:).'
        p = S.parent(t);
        if p == 0                          % root
            S.cost(t)    = 0;
            S.blocked(t) = false;
            S = recalcChildrenCost(t, S);  % will also unblock descendants
        elseif S.cost(p) < inf             % parent is reachable
            S.cost(t)    = S.cost(p) + costMetric(S.V(p,:), S.V(t,:), S.w);
            S.blocked(t) = false;
            S = recalcChildrenCost(t, S);
        else
            % parent still blocked -> leave as is (still blocked)
        end
    end
end