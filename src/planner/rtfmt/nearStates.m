function Y = nearStates(xIdx, stateDes, S)
    Nidx = near(S.V, S.V(xIdx,:), S.rn, S.w);
    mask = ismember(S.state(Nidx), stateDes);
    Y = Nidx(mask);
end

    