function S = addChild(child, parentIdx, totalCost, S)
    % Remove from W; attach to tree; openNew
    S.W(S.W == child) = [];
    S.parent(child)  = parentIdx;
    S.cost(child)    = totalCost;
    S.state(child)   = 1;       % Open
    S.isOpen(child)  = true;
    S.openNew(child) = true;              % keep your “openNew” bookkeeping

    if ~S.checkedPath(child)
        p = parentIdx;
        while p > 0
            if S.checkedPath(p)
                S.checkedPath(p) = false;
            end
            if isfield(S,'checkedPathCandidates') && ~isempty(S.checkedPathCandidates)
                S.checkedPathCandidates(S.checkedPathCandidates == p) = [];
            end
            p = S.parent(p);
            % defensive stop if there’s a cycle
            if p == parentIdx
                break
            end
        end
    end

end