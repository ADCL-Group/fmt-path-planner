function h = plotAnalyticalObst(obstacles)
% h = plotAnalyticObstacles(obstacles)
% Plots analytic obstacles defined as rows:
%   [x1, y1, z1,   x2, y2, z2,   radius, type]
%
% type = 0 -> cylinder along segment P1=(x1,y1,z1) to P2=(x2,y2,z2)
%            (covers vertical and inclined "tubes")
% type = 1 -> sphere centered at (x1,y1,z1), radius in column 7
% Output: handles of plotted objects.

    M = size(obstacles, 1);
    h = gobjects(M,1);

    nSide = 40;  % resolution for cylinder/sphere

    for i = 1:M
        x1 = obstacles(i,1);
        y1 = obstacles(i,2);
        z1 = obstacles(i,3);

        x2 = obstacles(i,4);
        y2 = obstacles(i,5);
        z2 = obstacles(i,6);

        r  = obstacles(i,7);
        type = obstacles(i,8);

        switch type
            case 1  % Sphere
                [Xs, Ys, Zs] = sphere(nSide);
                X = Xs * r + x1;
                Y = Ys * r + y1;
                Z = Zs * r + z1;

                h(i) = surf(X, Y, Z);

            case 0  % Cylinder along segment P1 -> P2
                P1 = [x1; y1; z1];
                P2 = [x2; y2; z2];
                v  = P2 - P1;
                L  = norm(v);

                % Unit direction along segment
                uz = v / L;

                % Local cylinder along +Z from 0 to L
                [Xc, Yc, Zc] = cylinder(r, nSide);
                Zc = Zc * L;  % height L

                % Stack into 3xN for rotation
                pts = [Xc(:)'; Yc(:)'; Zc(:)'];

                % Rotation taking [0;0;1] -> uz
                zAxis = [0; 0; 1];
                if norm(cross(zAxis, uz)) < 1e-12
                    % Already aligned or opposite
                    if dot(zAxis, uz) > 0
                        R = eye(3);   % same direction
                    else
                        % 180-degree flip around X (or any perpendicular axis)
                        R = [-1 0 0; 0 1 0; 0 0 -1];
                    end
                else
                    k = cross(zAxis, uz);
                    k = k / norm(k);
                    cosT = dot(zAxis, uz);
                    sinT = sqrt(1 - cosT^2);

                    K = [  0    -k(3)  k(2);
                          k(3)   0   -k(1);
                         -k(2)  k(1)  0  ];

                    R = eye(3) + sinT * K + (1 - cosT) * (K*K);
                end

                % Rotate and translate
                ptsRot = R * pts;
                ptsRot(1,:) = ptsRot(1,:) + P1(1);
                ptsRot(2,:) = ptsRot(2,:) + P1(2);
                ptsRot(3,:) = ptsRot(3,:) + P1(3);

                % Reshape back to mesh
                X = reshape(ptsRot(1,:), size(Xc));
                Y = reshape(ptsRot(2,:), size(Yc));
                Z = reshape(ptsRot(3,:), size(Zc));

                h(i) = surf(X, Y, Z);

            otherwise
                warning('Unknown obstacle type %d at row %d', type, i);
        end
    end
end
