function collides = segmentIntersectsSphere3D(pts, dynObs, R)
% pts    : P x 3 sampled points along Dubins/line path
% dynObs : M x 3 centers of dynamic spherical obstacles
% R      : safety radius

    collides = false;

    if isempty(pts) || isempty(dynObs)
        return;
    end

    % Extract coordinates
    X = pts(:,1);
    Y = pts(:,2);
    Z = pts(:,3);

    cx = dynObs(:,1).';
    cy = dynObs(:,2).';
    cz = dynObs(:,3).';

    dx = X - cx;
    dy = Y - cy;
    dz = Z - cz;

    dist2 = dx.^2 + dy.^2 + dz.^2;
    
    collides = any(dist2(:) <= R*R);
end
