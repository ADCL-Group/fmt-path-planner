function [intposes, totalCost, bestIdx] = interpDubins(conn, state1, state2, costs_y)
    % Create a Dubins path
    pathSegs = connect(conn, state1, state2);
    
    % Extract lengths and filter
    lengths = cellfun(@(seg) seg.Length, pathSegs);
    validIdx = find(~isnan(lengths));% indices into pathSegs
    
    % Pre‑allocate
    pathCosts = inf(size(state1,1), 1);
    
    for k = validIdx(:).'
        dist = lengths(k);
        pathCosts(k) = dist;
    end
    
    totalCosts = costs_y + pathCosts;
    
    [totalCost, bestIdx] = min(totalCosts);
    
    % Interpolate poses using Dubins motion primitives
    interval = 50/pathCosts(bestIdx);
    alpha = [0:interval:1 1];
    sampleDists = alpha * pathCosts(bestIdx);
    intposes = interpolate(pathSegs{bestIdx}, sampleDists);
end