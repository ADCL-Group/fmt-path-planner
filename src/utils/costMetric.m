function cost = costMetric(V,x,w)
% Given two samples u,x elements of V, and the weights w, Cost(V,x,w) 
% computes the distance between the two samples
    d = size(V,2);

    diffs3 = V(:,1:3) - x(1:3);
    dist2  = sum(diffs3.^2, 2);
    
    if d == 3
        cost = sqrt(dist2);
    elseif d == 4
        % heading difference, wrapped to [0, pi]
        rawPsi = abs(V(:,4) - x(4));
        dpsi = min(rawPsi, 2*pi - rawPsi);
    
        cost = sqrt( dist2 + (w * dpsi).^2 );
    end
end

