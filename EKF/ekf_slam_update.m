function [mu, Sigma, seen_landmarks] = ekf_slam_update(mu, Sigma, zt, sigma_r, sigma_phi, seen_landmarks)
% EKF SLAM measurement update
% Inputs:
%   mu             - state vector [x; y; theta; l1x; l1y; l2x; l2y; ...]
%   Sigma          - covariance matrix
%   zt             - measurements [r, phi, s] per row
%   sigma_r, sigma_phi - measurement noise std devs
%   seen_landmarks - logical array of initialized landmarks
% Outputs:
%   mu, Sigma      - updated state and covariance
%   seen_landmarks - updated flag array

    Qt = diag([sigma_r^2, sigma_phi^2]);  % 2x2 measurement noise

    for i = 1:size(zt, 1)
        z = zt(i,:)';       % [r; phi; s]
        r = z(1); phi = z(2); s = z(3);   % s = landmark ID

        j = s;  % assume ID directly maps to index
        idx = 3 + 2*(j-1) + 1;  % index of x_j in mu

        % --- Landmark Initialization ---
        if ~seen_landmarks(j)
            mu(idx)   = mu(1) + r * cos(phi + mu(3));
            mu(idx+1) = mu(2) + r * sin(phi + mu(3));
            seen_landmarks(j) = true;
        end

        % --- Innovation ---
        dx = mu(idx)   - mu(1);
        dy = mu(idx+1) - mu(2);
        q = dx^2 + dy^2;
        sqrt_q = sqrt(q);

        % Predicted measurement
        z_hat = [sqrt_q;
                 wrapToPi(atan2(dy, dx) - mu(3))];

        % Measurement difference
        z_obs = z(1:2);  % measured [r; phi]
        dz = z_obs - z_hat;
        dz(2) = wrapToPi(dz(2));

        % --- Jacobian H ∈ ℝ^{2 × len(mu)} ---
        H = zeros(2, length(mu));

        H(1,1) = -dx / sqrt_q;
        H(1,2) = -dy / sqrt_q;
        H(1,idx)   =  dx / sqrt_q;
        H(1,idx+1) =  dy / sqrt_q;

        H(2,1) =  dy / q;
        H(2,2) = -dx / q;
        H(2,3) = -1;
        H(2,idx)   = -dy / q;
        H(2,idx+1) =  dx / q;

        % --- Kalman Gain and Update ---
        S = H * Sigma * H' + Qt;
        K = Sigma * H' / S;

        mu = mu + K * dz;
        mu(3) = wrapToPi(mu(3));
        Sigma = (eye(length(mu)) - K * H) * Sigma;
    end
end
