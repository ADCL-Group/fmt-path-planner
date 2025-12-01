function S = recalcChildrenCost(seed, S, oldSeedCost)
    % When we rewire a single node, we can update the cost of the children
    % with a delta value (difference between new and old cost of the node)
    if nargin >= 3 && ~isempty(oldSeedCost)
        delta = S.cost(seed) - oldSeedCost;
        if abs(delta) < eps
            return;
        end

        Q = find(S.parent == seed);

        while ~isempty(Q)
            u = Q(1);
            Q(1) = [];

            if isfinite(S.cost(u))
                S.cost(u) = S.cost(u) + delta;
            end

            S.blocked(u) = isinf(S.cost(u));

            Q = [Q; find(S.parent == u)];
        end
        return;
    end

    % Recompute the cost of all the children
    Q = find(S.parent == seed);
    while ~isempty(Q)
        u = Q(1); 
        Q(1) = [];

        p = S.parent(u);

        if p == 0
            S.cost(u) = 0;   % should only happen at root
            S.blocked(u) = isinf(S.cost(u));
        else
            if isinf(S.cost(p))
                S.cost(u)    = inf;
                S.blocked(u) = true;
            else
                % Parent reachable: compute edge cost p -> u
                parentState = S.V(p,:);
                childState  = S.V(u,:);
    
                if ~isempty(S.conn)
                    % Dubins + heading penalty
                    aux1 = [0 0 0 parentState(4)];
                    aux2 = [0 0 0 childState(4)];
                    headingPenalty = costMetric(aux1, aux2, S.w);
    
                    [~, edgeLen, ~] = interpDubins(S.conn, parentState, childState, 0);
                    S.cost(u) = S.cost(p) + edgeLen + headingPenalty;
                else
                    % Straight-line
                    res = S.map.Resolution;
                    [~, edgeLen, ~] = interpLine(res, parentState, childState, 0);
                    S.cost(u) = S.cost(p) + edgeLen;
                end

                S.blocked(u) = isinf(S.cost(u));   % update blocked flag
            end
        end


        % push grandchildren
        Q = [Q; find(S.parent == u)];
    end
end