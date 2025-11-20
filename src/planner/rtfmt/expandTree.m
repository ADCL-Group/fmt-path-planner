function S = expandTree(S)
        % If Z’s neighbor list is empty and there are still unvisited nodes, refresh XNear for new z
        if ~isfield(S,'XNear') || isempty(S.XNear), S.XNear = []; end

        % If we have a z and XNear is empty, refresh it from W
        if ~isempty(S.W) && isempty(S.XNear) && ~isempty(S.z)
            Nz = near(S.V, S.V(S.z,:), S.rn, S.w);
            S.XNear = intersect(Nz(:), S.W(:));
        end

        % Try to connect one x from XNear this step
        if ~isempty(S.XNear)
            x = S.XNear(end); 
            S.XNear(end) = [];                          % pop last
        
            YNear = nearStates(x, 1, S);                % neighbors with state==Open
            [yMin, yCost] = argMinCost(x, YNear, false, S);
            if ~isempty(yMin)
                % Check fixed-obs free: straight or Dubins + occupancy tube
                isCFixed = isEdgeFixedFree(yMin, x, S);
                isCNodeOK = ~S.dynamicObstructed(x); %THIS IS NEW
                if (S.cost(yMin) < inf) && isCFixed && isCNodeOK && ~isDescendant(yMin, x, S) && (x ~= yMin)
                    S = addChild(x, yMin, yCost, S);
                end
            end
        end

        % When XNear empties, close z, push openNew, possibly move closed->open parents, then choose new z
        if isempty(S.XNear) && ~isempty(S.z)
            if any(S.openNew)
                S.isOpen(S.openNew) = true;
                S.state(S.openNew)  = 1;    % Open
                S.openNew(:) = false;
            end

            % close z:
            zClosed = S.z;
            S.isOpen(zClosed) = false;
            S.state(zClosed)  = 3;                                

            % If z has unvisited neighbors that are fixed-obs-free mark for reopen when dynamic obstacle moves
            zNbr = near(S.V, S.V(zClosed,:), S.rn, S.w);
            zNbr = intersect(zNbr(:), S.W(:));
            zHasCF = false;
            for k = zNbr(:).'
                if isEdgeFixedFree(zClosed, k, S)
                    zHasCF = true; 
                    break
                end
            end

            if zHasCF
                S.closedToOpen(zClosed) = true;
            end
                                       
            % new z = min-cost open (like findZ)
            openIdx = find(S.isOpen);
            if isempty(find(S.isOpen,1))
                S.z = []; 
            else
                [~,k] = min(S.cost(openIdx));
                S.z = openIdx(k);
            end
            S.XNear = [];
        end

        if isempty(S.z)
            % keep searching: reopen previously closed nodes if dynamic obstacles may have freed neighbors
            if any(S.closedToOpen)
                idx = find(S.closedToOpen);
                S.state(idx)  = 1;           % Open
                S.isOpen(idx) = true;        % reopen those
                S.closedToOpen(:) = false;
                
                openIdx = find(S.isOpen);
                if isempty(find(S.isOpen,1))
                    S.z = []; 
                else
                    [~,k] = min(S.cost(openIdx));
                    S.z = openIdx(k);
                end

                S.XNear = [];
            end
        end
    end

    