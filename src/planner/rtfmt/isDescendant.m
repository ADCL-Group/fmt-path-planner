function tf = isDescendant(candidateParent, x, S)
% True if candidateParent is in the subtree rooted at x

    tf = false;
    curr = candidateParent;

    % walk up the tree until we reach the root (parent == 0)
    while curr ~= 0
        if curr == x
            tf = true;
            return;
        end
        curr = S.parent(curr);
    end
end
