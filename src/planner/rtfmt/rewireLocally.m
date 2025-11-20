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

    % S.blocked(x) = false; % we’ll restore if fails
    % if ~isObstructed(x, S)
        YNear = nearStates(x, [1 3], S); % Open+Closed
        [yMin, yCost] = argMinCost(x, YNear, true, S);
        if ~isempty(yMin)
            isCFixed = isEdgeFixedFree(yMin, x, S);
            if S.cost(yMin) < inf && isCFixed && ~isDescendant(yMin, x, S) && x ~= yMin
                S.parent(x) = yMin; 
                S.cost(x) = yCost;                           % “addChildUpdateParent” equivalent  
                S = recalcChildrenCost(x, S);
                S.blocked(x) = false;
            else
                % S.blocked(x) = true; % still blocked
            end
        else
            % S.blocked(x) = true; % still blocked
        end
    % else
        % S.blocked(x) = true;
    % end
end

    
function tf = isInDynamicObstacle(i, S)
    % Mirror obstructedNodes1d.Contains(x) from C#
    % Provide S.dynamicObstructed(:) as a logical vector externally.
    tf = isfield(S,'dynamicObstructed') && ~isempty(S.dynamicObstructed) && S.dynamicObstructed(i);
end

