function r = optimalRadius(N, limits, w)
    dims = limits(:,2) - limits(:,1);   % dims = [dx; dy; dz, dpsi]
    volume = prod(dims);
    d = size(limits,1);
    
    eta = 1/d;

    if d == 3
        mu = volume;
        zeta = 4*pi/3;
    elseif d == 4
        mu = volume;
        zeta = (pi^2)/(2*w); % Based on the cost function
    end

    gamma = (1 + eta) * 2 * (1/d)^(1/d) * (mu/zeta)^(1/d);
    r = gamma * (log(N)/N)^(1/d);
end
