function tf = isEdgeFixedFree(i, j, S)

    % Static obstacle check 
    if ~isempty(S.conn)
        [pts,~,~] = interpDubins(S.conn, S.V(i,:), S.V(j,:), [0; 0]);
    else                            
        res = S.map.Resolution;
        [pts,~,~] = interpLine(res, S.V(i,:), S.V(j,:), [0; 0]);
    end
    occ = checkOccupancy(S.map, pts(:,1:3));
    tf_static  = all(occ < S.map.FreeThreshold);

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

    if segmentIntersectsSphere3D(pts, S.dynObs, S.safeRadiusDObstacle)
        tf = false; 
        return;
    end

    % pi = S.V(i,1:3); pj = S.V(j,1:3);
    % 
    % for k = 1:size(S.dynObs,1)
    %     c = S.dynObs(k,1:3);
    %     R = S.safeRadiusDObstacle;
    % 
    %     if segmentIntersectsSphere3D(pi, pj, c, R)
    %         tf = false; return;
    %     end
    % end

    tf = true;
end

