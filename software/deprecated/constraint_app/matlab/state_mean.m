function m = state_mean(x, w)

    % x is an Ns x N set of states, where the state dimension is N
    % w is a Ns x 1 vector of weights

    m = sum(x .* repmat(w, 1, size(x,2)), 1); %TODO: should do a lie algebra mean for x(:,1:6)