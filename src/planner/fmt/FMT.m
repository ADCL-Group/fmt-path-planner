function [traj, E, V] = FMT(map, limits, start, goal, N, rn, goalRadius, w, flightParams)
    % Fast Marching Tree Algorithm (FMT∗)
    V = sampleFree(map, limits, start, goal, N);
    allIdx  = 1:size(V,1);
    
    start_idx = find(ismember(V, start, 'rows'));
    goal_idx  = find(ismember(V, goal,  'rows'));
    
    % Determine the goal area
    [goalNbrIdx, ~] = near(V, V(goal_idx,:), goalRadius, w);
    goalRegionIdx = unique([goal_idx; goalNbrIdx]);
    
    W_idx = setdiff(allIdx, start_idx);  % Set to track nodes which have not been added to the tree (unvisited)
    H_idx = start_idx; % Set to track nodes which have already been added to the tree but are open to connections (open)
    z_idx = start_idx;
    [Nz_idx, ~] = near(V, V(z_idx,:), rn, w);

    vertexStore = struct();
    vertexStore.parent = zeros(N,1);
    vertexStore.vertex = zeros(N, size(V,2));
    vertexStore.neighbors = cell(N,1);
    vertexStore.cost = inf(N,1);

    vertexStore.vertex(start_idx,:) = V(z_idx,:);
    vertexStore.neighbors{start_idx} = Nz_idx;
    vertexStore.cost(start_idx) = 0;

    if nargin == 9
        % Create the Dubins connector
        conn = uavDubinsConnection( ...
            'MaxRollAngle', flightParams(1), ...
            'AirSpeed', flightParams(2), ...
            'FlightPathAngleLimit', flightParams(3:4));
    end

    freeThresh = map.FreeThreshold;
    res = map.Resolution;

    % Main FMT Loop
    while ~ismember(z_idx, goalRegionIdx)
        Xnear_idx = intersect(Nz_idx, W_idx);
        Hnew_idx = zeros(1, numel(Xnear_idx));
        hcount = 0;
    
        for xi = Xnear_idx(:).'
            [Nx_idx, ~] = near(V, V(xi,:), rn, w);

            Ynear = intersect(Nx_idx, H_idx);
            if isempty(Ynear)
                continue;
            end
            costs_y = [vertexStore.cost(Ynear)];

            if nargin == 9
                % Vectorized Dubins primitives

                % Compute heading difference penalty
                aux1 = [zeros(size(V(Ynear,1:3))) V(Ynear,4)];
                aux2 = [zeros(size(V(xi,1:3))) V(xi,4)];
                headingPenalty = costMetric(aux1, aux2, w);
                cost = costs_y + headingPenalty;

                goals = repmat(V(xi,:), size(Ynear,1), 1);
                [intposes, totalCost, k] = interpDubins(conn, V(Ynear,:), goals, cost);
            else
                % Straight-line case
                pathCosts = costMetric(V(Ynear,:), V(xi,:), w);
                cost = costs_y + pathCosts;
                [intposes, totalCost, k] = interpLine(res, V(Ynear,:), V(xi,:), cost);
            end

            % Occupancy check
            % MATLAB checks by doing ~isOccupied which is more restrictive 
            % because unknown spaces (-1) in the map are marked as occupied. 
            % Checking if the value is less than the map's threshold sets 
            % the unknown as free.
            isOccupied = map.checkOccupancy(intposes(:,1:3));
            
            if all(isOccupied < freeThresh)
                % Update tree
                vertexStore.parent(xi) = Ynear(k);
                vertexStore.cost(xi) = totalCost;
                vertexStore.vertex(xi,:) = V(xi,:);
                vertexStore.neighbors{xi} = Nx_idx;
    
                hcount = hcount + 1;
                Hnew_idx(hcount) = xi;
                W_idx = setdiff(W_idx, xi);
            end
        end
        % Update sets
        Hnew_idx = Hnew_idx(1:hcount);
        H_idx = setdiff(union(H_idx, Hnew_idx), z_idx);
    
        if isempty(H_idx)
            error('FMT* failed to find any way to reach the goal.');
        end

        [~, j] = min([vertexStore.cost(H_idx)]);
        z_idx  = H_idx(j);
    
        if ~isempty(vertexStore.neighbors{z_idx})
            Nz_idx = vertexStore.neighbors{z_idx};
        else
            [Nz_idx, ~] = near(V, V(z_idx,:), rn, w);
        end
    end
    
    % Build the tree

    childIdx = find(vertexStore.parent > 0); % Find all vertex xi that got a parent
    E = [vertexStore.parent(childIdx), childIdx];
    
    idxPath = generatePathBackward(z_idx, vertexStore);
    traj = V(idxPath, :);
end