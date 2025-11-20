function [reached, gHit] = isInTree(goalSet, S)
        
    goalSet = goalSet(:);

    inTreeMask = (S.cost(goalSet) < inf) & ( (S.parent(goalSet) ~= 0) | (goalSet == S.rootIdx) );
    hits = goalSet(inTreeMask);

    if isempty(hits)
        reached = false; gHit = [];
        return
    end

    % Choose the best hit; default = lowest cost-to-come
    [~,k] = min(S.cost(hits));
    gHit = hits(k);
    reached = true;
end

    