function idxPath = generatePathBackward(g, S)
    % Follow parents from goal to root and reverse
    % ASSUME: parent(rootIdx)=0; costs set; parents valid along the chain
    N = numel(S.parent);
    buffer = zeros(N, 1);         % preallocate
    ptr = N;
    curr = g;

    while curr ~= 0
        buffer(ptr) = curr;
        curr = S.parent(curr);
        ptr = ptr - 1;
    end
    
    % Slice only the used part (from ptr+1 to end)
    idxPath = buffer(ptr+1:end);
end

    