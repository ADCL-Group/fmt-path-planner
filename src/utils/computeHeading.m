function psi = computeHeading(trajXY)
    % extract just the XY columns
    xy = trajXY(:,1:2);  
    N  = size(xy,1);
    psi = zeros(N,1);

    for i = 1:N
        if i==1
            % forward difference at the start
            delta = xy(2,:) - xy(1,:);
        elseif i==N
            % backward difference at the end
            delta = xy(N,:) - xy(N-1,:);
        else
            % central difference in the middle
            delta = (xy(i+1,:) - xy(i-1,:)) / 2;
        end
        psi(i) = atan2(delta(2), delta(1));
    end
end
