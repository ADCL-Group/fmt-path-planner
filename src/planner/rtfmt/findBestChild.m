function y = findBestChild(c, S)
    % C# hides this in a method. A practical stand-in is:
    % 1) children = nodes whose parent==c and cost<INF
    % 2) pick the child closest to the goal (consistent with their downstream comparison)
    children = find(S.parent==c & S.cost<inf);
    if isempty(children)
        y = [];
        return
    end

    if isfield(S, 'checkedPath') && ~isempty(S.checkedPath)
        children = children(~S.checkedPath(children));
    end

    if isempty(children)
        y = [];   % all children are blocked
        return
    end

    % C# uses f = g + h, where g = S.cost(child), h = ||child - goal||
    h = vecnorm(S.V(children,:) - S.V(S.goalIdx,:), 2, 2);
    f = S.cost(children) + h;

    % choose argmin f
    [~, k] = min(f);
    y = children(k);
end

    