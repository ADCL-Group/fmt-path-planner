function [idx, neighbors] = near(V, x, r, w)
% Compute the neighbors of a vertex inside a radius r 
% Given a set of vertices V and a positive number r, let Near(V, x, r) 
% return the set of vertices v elements of V where: Cost(v,x) < r}
    dAll = costMetric(V, x, w);
    idx = find(dAll < r);
    neighbors = V(idx, :);
end

