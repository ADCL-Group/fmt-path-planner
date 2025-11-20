function S = rewireFromRoot2(S)
    if ~isfield(S,'rewireRootList') || isempty(S.rewireRootList)
        S.rewireRootList = S.rootIdx;                 % seed with root
        S.rewireRootSeen = false(S.N,1);
        S.rewireRootSeen(:) = false;
        S.rewireRootSeen(S.rootIdx) = true;
    end

    % Pop first
    xChild = S.rewireRootList(1);
    S.rewireRootList(1) = [];

    % Candidate parents: Open or Closed neighbors of xChild
    YParents = nearStates(xChild, [1 3], S);

    % Try reparenting xChild to each parent y
    cOld = S.cost(xChild);
    for k = 1:numel(YParents)
        y = YParents(k);

        % --- match C# obstruction logic: check CHILD only ---
        % collisionFreeDynamic = ~isObstructed(xChild, S); % child-only
        collisionFreeDynamic = ~S.dynamicObstructed(xChild);   % child-only
        collisionFreeFixed   = isEdgeFixedFree(y, xChild, S);

        if collisionFreeDynamic && collisionFreeFixed
            cNew = S.cost(y) + costMetric(S.V(y,:), S.V(xChild,:), S.w); 
            if cNew < cOld && ~isDescendant(y, xChild, S) && xChild ~= y
                % detach from previous parent and reparent
                % oldp = S.parent(xChild);
                S.parent(xChild) = y;
                S.cost(xChild)   = cNew;
                S.blocked(xChild)= false;
                S = recalcChildrenCost(xChild, S);
                cOld = cNew; % keep best
            end
        end

        % --- frontier expansion: add y if not seen ---
        if ~S.rewireRootSeen(y)
            S.rewireRootList(end+1) = y;
            S.rewireRootSeen(y) = true;
        end
    end
end

    