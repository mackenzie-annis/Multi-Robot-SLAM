function [mu, Sigma] = ekf_motion_update(mu_prev, Sigma_prev, u, R_t, dt, N)
% EKF motion update using velocity-based unicycle model
% Inputs:
%   mu_prev     - previous state mean [x; y; theta; landmarks...]
%   Sigma_prev  - previous covariance matrix
%   u           - control input [v; w]
%   R_t         - motion noise (3x3)
%   dt          - time step
%   N           - number of landmarks
% Outputs:
%   mu          - predicted state mean
%   Sigma       - predicted covariance

    v = u(1);
    w = u(2);
    theta = mu_prev(3);
    epsilon = 1e-6;

    % Compute motion delta
    if abs(w) < epsilon
        delta = [
            v * cos(theta) * dt;
            v * sin(theta) * dt;
            0
        ];
        
        Gx = [
            1, 0, -v * sin(theta) * dt;
            0, 1,  v * cos(theta) * dt;
            0, 0, 1
        ];
    else
        delta = [
            -(v/w) * sin(theta) + (v/w) * sin(theta + w*dt);
             (v/w) * cos(theta) - (v/w) * cos(theta + w*dt);
             w * dt
        ];
        
        Gx = [
            1, 0, -(v/w) * cos(theta) + (v/w) * cos(theta + w*dt);
            0, 1, -(v/w) * sin(theta) + (v/w) * sin(theta + w*dt);
            0, 0, 1
        ];
    end

    % Build full Jacobian G_t for entire state vector
    Fx = [eye(3), zeros(3, 2*N)];
    G_t = eye(3 + 2*N) + Fx' * (Gx - eye(3)) * Fx;

    % Update mean
    mu = mu_prev + Fx' * delta;
    % Update covariance
    Sigma = G_t * Sigma_prev * G_t' + Fx' * R_t * Fx;
end
