function [F_mo, F_xmo, F_x, m_plus, m_zero] = build_SEIF_projection_matrices(mu, N, observed_landmarks, distance_threshold)
% Inputs:
% mu: current state vector [x; y; theta; landmarks...]
% N: number of landmarks
% observed_landmarks: array of landmark indices currently seen
% distance_threshold: distance above which landmarks are sparsified
% m0 matrix
% Outputs:
% F_mo  = selector for landmarks to deactivate
% F_xmo = selector for robot + actuve landmarks
% F_x   = selector for full state 
% m_plus, m_zero = landmark indices selected

    state_len = length(mu);
    robot_idx = 1:3;

    m_plus = [];  % to keep
    m_zero = [];  % to sparsify

    % Classify landmarks
    for j = 1:N
        xj_idx = 3 + 3*(j-1) + 1;
        yj_idx = xj_idx + 1;
        xj = mu(xj_idx);
        yj = mu(yj_idx);

        dist = norm([xj - mu(1); yj - mu(2)]);

        if ismember(j, observed_landmarks) || dist < distance_threshold
            m_plus = [m_plus, j];
        else
            m_zero = [m_zero, j];
        end
    end

    landmark_indices = @(js) reshape([3 + 3*(js - 1) + 1; ...
                                      3 + 3*(js - 1) + 2; ...
                                      3 + 3*(js - 1) + 3], 1, []);

    % F_mo
    idx_mo = [robot_idx, landmark_indices(m_zero)];
    F_mo = build_selector(idx_mo, state_len);

    % F_xmo
    idx_xmo = [robot_idx, landmark_indices([m_plus m_zero])];
    F_xmo = build_selector(idx_xmo, state_len);

    % F_x
    F_x = build_selector(1:state_len, state_len);
end


function F = build_selector(indices_to_keep, state_length)
    num_selected = length(indices_to_keep);
    F = sparse(num_selected, state_length);
    for i = 1:num_selected
        F(i, indices_to_keep(i)) = 1;
    end
end