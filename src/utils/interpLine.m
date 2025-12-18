function [intposes, totalCost, bestIdx] = interpLine(res, state1, state2, costs_y, pad)
% Given two vertices x and z in Xfree, return true if and only if xz does not intersect an obstacle

    if nargin < 5
        pad = 0;
    end

    % Candidate parent positions and child position
    Xparents = state1(:, 1:3);     % [nParents x 3]
    Z = state2(1, 1:3);     % [1 x 3]

    % Straight-line distances for all nodes
    diffs = Xparents - Z;        
    edgeLens = sqrt(sum(diffs.^2, 2));

    totalCosts = costs_y + edgeLens;

    % Pick best connection
    [totalCost, bestIdx] = min(totalCosts);

    x = Xparents(bestIdx, :);
    z = Z;

    dist = edgeLens(bestIdx);

    % Sampling density
    dx = 1 / res;                % unit per voxel
    ptsPerVoxel = 5;
    nPts = max(2, ceil(dist/dx * ptsPerVoxel));

    t = linspace(0, 1, nPts)';
    pts = (1 - t) * x + t * z;

    % Optional padding tube around the line
    if pad > 0
        offsets = pad * [ 1  0  0;
                         -1  0  0;
                          0  1  0;
                          0 -1  0;
                          0  0  1;
                          0  0 -1 ];

        nOff = size(offsets, 1);
        allCnt = (1 + nOff) * nPts; % Padding for each point (+- in X,Y,Z)
        intposes = zeros(allCnt, 3);

        % center line first
        intposes(1:nPts, :) = pts;

        % offset "tube" lines
        for k = 1:nOff
            idxStart = k*nPts + 1;
            idxEnd = (k+1)*nPts;
            intposes(idxStart:idxEnd, :) = pts + offsets(k, :);
        end
    else
        intposes = pts;
    end
end
