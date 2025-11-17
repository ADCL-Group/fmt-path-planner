function [intposes, totalCost, bestIdx] = interpLine(res, state1, state2, costs_y, pad)
% Given two vertices x and z in Xfree, return true if and only if xz does not intersect an obstacle

    [totalCost, bestIdx] = min(costs_y);
  
    if nargin < 5
        pad = 0;
    end

    x = state1(bestIdx, 1:3);
    z = state2(1, 1:3);

    dist  = norm(x - z);
    
    dx    = 1/res;              % unit per voxel
    
    % 'Draw' a line of at least 2 points between x and z
    ptsPerVoxel = 5;
    nPts = max(2, ceil(dist/dx*ptsPerVoxel));
    t     = linspace(0,1,nPts)';
    pts   = (1-t)*x + t*z;

    if pad > 0
        offsets = pad * [ 1  0  0;
                         -1  0  0;
                          0  1  0;
                          0 -1  0;
                          0  0  1;
                          0  0 -1 ];

        nOff    = size(offsets,1);
        allCnt  = (1 + nOff)*nPts; % Padding for each point (+- in X,Y,Z)
        intposes  = zeros(allCnt,3);

        % center line first
        intposes(1:nPts, :) = pts;

        % then each offset “tube”
        for k = 1:nOff
            idxStart = k*nPts + 1;
            idxEnd   = (k+1)*nPts;
            intposes(idxStart:idxEnd, :) = pts + offsets(k,:);
        end

    else
        intposes = pts;
    end
end
