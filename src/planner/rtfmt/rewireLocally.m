function S = rewireLocally(S)
    % Try to re-parent blocked children to better open/closed neighbors (local clean-up)  
    if ~isfield(S,'rewireLocalList') || isempty(S.rewireLocalList)
        S.rewireLocalList = find(S.blocked);
    end
    if isempty(S.rewireLocalList), return; end
    
    x = S.rewireLocalList(1);  S.rewireLocalList(1) = [];% pop first

    % Skip only if the node is currently inside a dynamic-obstacle zone
    if isInDynamicObstacle(x, S)
        % In the C# code nothing else happens here; leave S.blocked(x) as-is.
        return
    end

    YNear = nearStates(x, [1 3], S); % Open+Closed

    [yMin, yCost, isCFixed] = selectBestParent(x, YNear, S);
    if ~isempty(yMin) && isCFixed
        if S.cost(yMin) < inf && ~isDescendant(yMin, x, S) && x ~= yMin
            oldSeedCost = S.cost(x);

            S.parent(x) = yMin; 
            S.cost(x) = yCost;                           % “addChildUpdateParent” equivalent  
            S = recalcChildrenCost(x, S, oldSeedCost);
            S.blocked(x) = false;
        end
    end
end

    
function tf = isInDynamicObstacle(i, S)
    % Mirror obstructedNodes1d.Contains(x) from C#
    % Provide S.dynamicObstructed(:) as a logical vector externally.
    tf = isfield(S,'dynamicObstructed') && ~isempty(S.dynamicObstructed) && S.dynamicObstructed(i);
end

