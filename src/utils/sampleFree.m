function nodes = sampleFree(map3D, limits, start, goal, N)
% Function that returns a set of N points sampled independently and 
% identically from the uniform distribution on Xfree

    batchSize = max(100, N);
    d = size(limits,1);
    
    % Define the limits of the map
    spanVals = limits(:,2) - limits(:,1);
    
    samples = zeros(N, d);

    assert(checkOccupancy(map3D, start(1:3)) < map3D.FreeThreshold, 'Start is not free');
    assert(checkOccupancy(map3D, goal(1:3))  < map3D.FreeThreshold, 'Goal  is not free');
    
    count = 0;
    while count < N
        % Get a 3D coordinate within the map
        M = batchSize;
        randUniform = rand(M, d);

        pts = zeros(M, d);
        for i = 1:d
            pts(:,i) = spanVals(i)*randUniform(:,i) + limits(i,1);
        end

        % Check if the candidate nodes are free space
        occ = checkOccupancy(map3D, pts(:,1:3));
        freeIdx = occ < map3D.FreeThreshold;
        freePts = pts(freeIdx, :);

        nFree = size(freePts,1);
        nUsable = min(nFree, N - count);

        if nUsable > 0
            samples(count + (1:nUsable), :) = freePts(1:nUsable, :);
            count = count + nUsable;
        end
    end

    % Add start, goal and the N samples
    nodes = [ start; samples; goal ];
end