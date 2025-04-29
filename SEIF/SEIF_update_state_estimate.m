function mu = SEIF_update_state_estimate(xi, omega, mu, active_ids)
% Inputs:
% xi:        information vector (column vector)
% omega:     information matrix (sparse or full)
% mu:        current state estimate (3 + 3N x 1)
% active_ids: indices of landmarks to update 
    state_len = length(mu);
    % Update map features
    if ~isempty(active_ids)
        for idx = 1:length(active_ids)
            i = active_ids(idx);  % i scalar
            landmark_idx = 3 + 3*(i-1) + (1:3);  % x, y, s indices 1x3
            F_i = sparse(3, length(mu));
            F_i(1, landmark_idx(1)) = 1;
            F_i(2, landmark_idx(2)) = 1;
            F_i(3, landmark_idx(3)) = 1;
            A = F_i * omega * F_i';
            B = xi - omega * mu + omega * F_i' * F_i * mu;
            mu(landmark_idx) = inv(A) * F_i * B;
        end
    end

    % For all other features, keep previous estimate
    % Update robot pose (x, y, theta)
    F_x = sparse(3, state_len);
    F_x(1, 1) = 1;
    F_x(2, 2) = 1;
    F_x(3, 3) = 1;
    
    % Robot mu
    C = F_x * omega * F_x';
    D = xi - omega * mu + omega * F_x' * F_x * mu;
    mu(1:3) = C \ F_x * D;
end
