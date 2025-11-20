function tf = isEdgeFixedFree(i, j, S)

    % Static obstacle check                             
    res = S.map.Resolution;
    [pts,~,~] = interpLine(res, S.V(i,:), S.V(j,:), [0; 0]);
    occ = checkOccupancy(S.map, pts(:,1:3));
    % tf  = all(occ < map.FreeThreshold);   
    tf_static  = all(occ < 0.2);

    if ~tf_static
        tf = false; return;
    end

    if ~isfield(S,'dynObs') || isempty(S.dynObs)
        tf = true; return;
    end

    % If either endpoint is already flagged as dynamically obstructed, block edge.
    if S.dynamicObstructed(i) || S.dynamicObstructed(j)
        tf = false; return;
    end

    pi = S.V(i,1:3); pj = S.V(j,1:3);

    for k = 1:size(S.dynObs,1)
        c = S.dynObs(k,1:3);
        R = S.safeRadiusDObstacle;

        if segmentIntersectsSphere3D(pi, pj, c, R)
            tf = false; return;
        end
    end

    tf = true;
end

function tf = segmentIntersectsSphere3D(a, b, c, R)
% True if the segment ab touches the sphere (c, R)
    ab = b - a;
    ac = c - a;
    L2 = dot(ab, ab);
    if L2 == 0
        d2 = dot(ac, ac);
        tf = d2 <= R*R;
        return;
    end
    t  = max(0, min(1, dot(ac, ab) / L2));
    q  = a + t * ab;
    d2 = sum((q - c).^2);
    tf = d2 <= R*R;
end