function limits2 = expandLimits(limits, pts, pad)

    if nargin < 3 || isempty(pad), pad = 0; end

    mins = min(pts,[],1)';   % 3x1
    maxs = max(pts,[],1)';   % 3x1
    desired = [mins - pad, maxs + pad];   % 3x2

    limits2 = limits;
    limits2(:,1) = min(limits(:,1), desired(:,1));   % lower bounds
    limits2(:,2) = max(limits(:,2), desired(:,2));   % upper bounds
end