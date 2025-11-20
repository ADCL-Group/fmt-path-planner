function S = recalcChildrenCost(seed, S)
    % Match C#: ONLY children and descendants of seed (not seed itself)
    kids = find(S.parent == seed);
    Q = kids(:);
    while ~isempty(Q)
        u = Q(1); Q(1) = [];
        p = S.parent(u);

        % parent must exist; compute cost from parent
        if p == 0
            S.cost(u) = 0;
        else
            S.cost(u) = S.cost(p) + costMetric(S.V(p,:), S.V(u,:), S.w);
        end

        % Update blocked mask from INF
        if isinf(S.cost(u))
            S.blocked(u) = true;
        else
            S.blocked(u) = false;
        end

        % Push further
        Q = [Q; find(S.parent == u)];
    end
end