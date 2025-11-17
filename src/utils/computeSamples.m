function N = computeSamples(w, limits, alpha)
% computeSamples  Compute number of uniformly sampled waypoints in 3D.
%   - w: vehicle turn radius (scalar)
%   - limits: 3x2 array, each row [min max] for x, y, z
%   - alpha: (optional) spacing factor, default = 1


    if nargin < 3
        alpha = 1; % default spacing factor
    end
    
    L = limits(:,2) - limits(:,1);
    s = alpha * w; % Grid spacing
    
    % Number of samples per axis
    Nx = ceil(L(1)/s) + 1;
    Ny = ceil(L(2)/s) + 1;
    Nz = ceil(L(3)/s) + 1;

    N = Nx * Ny * Nz; % Total number of samples
end
