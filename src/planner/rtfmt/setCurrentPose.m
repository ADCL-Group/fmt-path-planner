function S = setCurrentPose(q, S)

    if ~isfield(S,'lastRootIdx'), S.lastRootIdx = S.rootIdx; end
    oldRoot = S.rootIdx;

    % Find nearest vertex
    finiteMask = S.cost(:) < inf;
    if any(finiteMask)
        xyz = S.V(finiteMask, 1:3);
        d2  = sum( (xyz - q(1:3)).^2, 2 );
        [~, rel] = min(d2);
        % map back to absolute index
        finiteIdx = find(finiteMask);
        iNear = finiteIdx(rel);
        dNear = sqrt(d2(rel));
    else
        iNear = [];
        dNear = inf;
    end

    % If it's far, insert the current pose as a new vertex
    insertThresh = S.rn;                    % tunable
    if isempty(iNear) || dNear > insertThresh
        newIdx = size(S.V,1) + 1;

        S.V(newIdx, :)   = q;
        S.parent(newIdx) = 0;
        S.cost(newIdx)   = 0;
        S.state(newIdx)  = 1;          % Open

        % S.isOpen(:) = false;
        % S.isOpen(newIdx) = true;

        if ~isempty(oldRoot)
            S.parent(oldRoot) = newIdx;
        end
        S.rootIdx = newIdx;

        S.blocked(newIdx) = false;
        S.openNew(newIdx) = false;
        S.closedToOpen(newIdx) = false;
        S.dynamicObstructed(newIdx) = false;
        S.dynamicObstructedPrev(newIdx) = false;
        S.rewireRootSeen(newIdx) = false;
        S.N = newIdx;

    else
        newRoot = iNear;

        if newRoot ~= oldRoot
            S = reversePathToRoot(S, newRoot, oldRoot);
            S.rootIdx = newRoot;
            S.cost(newRoot) = 0;   % baseline
        end
        
        S.parent(S.rootIdx) = 0;      % root has no parent
        S.cost(S.rootIdx)   = 0;
        S.state(S.rootIdx) = 1; % Open
        
        % S.isOpen(:) = false;
        % S.isOpen(S.rootIdx) = true;
    end

    S.z  = S.rootIdx;
    S.Nz = near(S.V, S.V(S.z, :), S.rn, S.w);

    if ~isfield(S, 'rewireRootList'), S.rewireRootList = []; end

    % 3) Recompute subtree costs from the new root and try local rewiring
    S = recalcChildrenCost(S.rootIdx, S);
    S = rewireFromRoot2(S);

    S.lastRootIdx = S.rootIdx;
end

function S = reversePathToRoot(S, newRoot, oldRoot)
    % Reverse parent pointers along the unique path from newRoot up to oldRoot,
    % so that newRoot becomes the tree root (parent==0) and the old root becomes
    % a descendant. Works even if the path is longer than one edge.
    prev = 0;
    cur  = newRoot;

    % Guard against degenerate/malformed parent arrays
    maxHops = numel(S.parent) + 1;

    for k = 1:maxHops
        next = S.parent(cur); % old parent
        S.parent(cur) = prev; % reverse link

        if cur == oldRoot
            break
        end

        prev = cur;
        cur  = next;

        % If we ever hit 0 before reaching oldRoot, the tree was split; stop.
        if cur == 0
            break
        end
    end
end