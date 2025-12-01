function S = rewireFromRoot2(S)
% Explore all parents and keep the best feasible one
    % Initialize rewire frontier from root if needed
    if ~isfield(S,'rewireRootList') || isempty(S.rewireRootList)
        S.rewireRootList = S.rootIdx;      % seed with root
        S.rewireRootSeen = false(S.N,1);
        S.rewireRootSeen(:) = false;
        S.rewireRootSeen(S.rootIdx) = true;
    end

    % Pop first node to rewire
    xChild = S.rewireRootList(1);
    S.rewireRootList(1) = [];

    % Candidate parents: Open or Closed neighbors of xChild
    YParents = nearStates(xChild, [1 3], S);

    cOld = S.cost(xChild);

    for k = 1:numel(YParents)
        y = YParents(k);

        collisionFreeDynamic = ~S.dynamicObstructed(xChild);
        if ~collisionFreeDynamic
            continue;
        end

        parentState = S.V(y,:);
        childState  = S.V(xChild,:);

        % Compute candidate cost and interpolated path
        if ~isempty(S.conn)
            % Dubins case

            aux1 = [0 0 0 parentState(4)];
            aux2 = [0 0 0 childState(4)];
            headingPenalty = costMetric(aux1, aux2, S.w);

            [pts, edgeCost, ~] = interpDubins(S.conn, parentState, childState, 0);
            cNew = S.cost(y) + edgeCost + headingPenalty;
        else
            % Straight-line
            res = S.map.Resolution;

            [pts, edgeLen, ~] = interpLine(res, parentState, childState, 0);
            cNew = S.cost(y) + edgeLen;
        end

        % Static obstacle check
        occ = checkOccupancy(S.map, pts(:,1:3));
        tf_static = all(occ < S.map.FreeThreshold);
        if ~tf_static
            continue;
        end

        % Dynamic obstacle checks
        if isfield(S,'dynObs') && ~isempty(S.dynObs)
            % If either endpoint is already dynamically obstructed, block edge
            if S.dynamicObstructed(y) || S.dynamicObstructed(xChild)
                continue;
            end

            if segmentIntersectsSphere3D(pts, S.dynObs, S.safeRadiusDObstacle)
                continue;
            end

            % pi = S.V(y,1:3);
            % pj = S.V(xChild,1:3);
            % 
            % blockedByDyn = false;
            % for d = 1:size(S.dynObs,1)
            %     c = S.dynObs(d,1:3);
            %     R = S.safeRadiusDObstacle;
            % 
            %     if segmentIntersectsSphere3D(pi, pj, c, R)
            %         blockedByDyn = true;
            %         break;
            %     end
            % end
            % if blockedByDyn
            %     continue;
            % end
        end

        % If cost improves, reparent
        if (cNew < cOld) && ~isDescendant(y, xChild, S) && (xChild ~= y)
            oldSeedCost = S.cost(xChild);

            S.parent(xChild) = y;
            S.cost(xChild) = cNew;
            S.blocked(xChild) = false;
            S = recalcChildrenCost(xChild, S, oldSeedCost);
            cOld = cNew;   % update best cost
        end

        % --- frontier expansion: add y if not seen ---
        if ~S.rewireRootSeen(y)
            S.rewireRootList(end+1) = y;
            S.rewireRootSeen(y) = true;
        end
    end
end
