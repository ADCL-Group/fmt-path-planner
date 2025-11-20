function inside = isInsideObstacles(points, obstacles)
% points    : N x 3  matrix of [x, y, z] query points
% obstacles : M x 8  matrix of analytic obstacles:
%   [x1, y1, z1,   x2, y2, z2,   radius, type]
%
%   type = 0 -> finite cylinder / capsule along segment P1->P2
%              (vertical or inclined; radius is constant)
%   type = 1 -> sphere, center = (x1,y1,z1), radius in col 7
%
% inside(i) : true if point i is inside at least one obstacle

    % if isvector(points) && numel(points) == 3
    %     points = reshape(points, 1, 3);
    % end

    N = size(points, 1);
    inside = false(N, 1);

    % Early exit if no obstacles
    if isempty(obstacles)
        return;
    end

    % Split obstacle types
    TYPE_CYLINDER = 0;
    TYPE_SPHERE   = 1;

    isCyl  = obstacles(:, 8) == TYPE_CYLINDER;
    isSph  = obstacles(:, 8) == TYPE_SPHERE;

    % Extract point coordinates
    X = points(:, 1);   % N x 1
    Y = points(:, 2);   % N x 1
    Z = points(:, 3);   % N x 1

    % Spheres
    if any(isSph)
        sphObs  = obstacles(isSph, :);      % Ns x 8
        cx_s    = sphObs(:, 1).';           % 1 x Ns
        cy_s    = sphObs(:, 2).';           % 1 x Ns
        cz_s    = sphObs(:, 3).';           % 1 x Ns
        r_s     = sphObs(:, 7).';           % 1 x Ns

        % Implicit expansion: N x 1 minus 1 x Ns -> N x Ns
        dx = X - cx_s;
        dy = Y - cy_s;
        dz = Z - cz_s;

        dist2 = dx.^2 + dy.^2 + dz.^2;      % N x Ns
        insideSpheres = any(dist2 <= r_s.^2, 2);  % N x 1

        inside = inside | insideSpheres;
    end

    % Cylinders (z in [0, zmax])
    if any(isCyl)
        cylObs  = obstacles(isCyl, :);      % Nc x 8

        Ax = cylObs(:, 1).';   % 1 x Nc
        Ay = cylObs(:, 2).';
        Az = cylObs(:, 3).';

        Bx = cylObs(:, 4).';
        By = cylObs(:, 5).';
        Bz = cylObs(:, 6).';

        R  = cylObs(:, 7).';   % radii, 1 x Nc

        % Vectors AB (segment direction)
        ABx = Bx - Ax;         % 1 x Nc
        ABy = By - Ay;
        ABz = Bz - Az;

        AB2 = ABx.^2 + ABy.^2 + ABz.^2;

        % For each point P, compute projection on each segment
        % AP = P - A
        APx = X - Ax;          % N x Nc (implicit expansion)
        APy = Y - Ay;
        APz = Z - Az;

        t = (APx .* ABx + APy .* ABy + APz .* ABz) ./ AB2;  % N x Nc
        t = max(0, min(1, t));

        % Closest point C = A + t * AB
        % Vector from P to C: PC = AP - t*AB
        PCx = APx - t .* ABx;      % N x Nc
        PCy = APy - t .* ABy;
        PCz = APz - t .* ABz;

        dist2_PC = PCx.^2 + PCy.^2 + PCz.^2;     % N x Nc

        insideCapsules = any(dist2_PC <= R.^2, 2);  % N x 1

        inside = inside | insideCapsules;

        % cx_c    = cylObs(:, 1).';           % 1 x Nc
        % cy_c    = cylObs(:, 2).';           % 1 x Nc
        % zmax_c  = cylObs(:, 3).';           % 1 x Nc
        % r_c     = cylObs(:, 4).';           % 1 x Nc
        % 
        % dx = X - cx_c;                      % N x Nc
        % dy = Y - cy_c;                      % N x Nc
        % 
        % dist2_xy = dx.^2 + dy.^2;           % N x Nc
        % inCircle = dist2_xy <= r_c.^2;      % N x Nc
        % 
        % % Height: z in [0, zmax]
        % % Z: N x 1, zmax_c: 1 x Nc -> N x Nc via implicit expansion
        % inHeight = (Z >= 0) & (Z <= zmax_c);
        % 
        % insideCylinders = any(inCircle & inHeight, 2);  % N x 1
        % 
        % inside = inside | insideCylinders;
    end
end
