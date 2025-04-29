function [xi, omega, mu] = SEIF_motion_update(xi_prev, omega_prev, mu_prev, u, Rt, dt, N)
    % Inputs:
    % xi_prev: previous information vector
    % Omega_prev: previous information matrix
    % mu_prev: previous mean state [x; y; theta; landmarks...]
    % u: control input [v; omega]
    % Rv: motion noise covariance (3x3)
    % dt: time step
    % N: num landmarks

    v = u(1);
    w = u(2);
    theta = mu_prev(3);
    Fx = [eye(3), zeros(3, 3*N)];

    epsilon = 1e-6;  % small threshold to avoid division by zero

    if abs(w) < epsilon
        % Straight-line motion
        delta = [
            v * cos(theta) * dt;
            v * sin(theta) * dt;
            0
        ];
        
        Delta = [
            0, 0, -v * sin(theta) * dt;
            0, 0,  v * cos(theta) * dt;
            0, 0, 0
        ];
    else
        % General unicycle motion
        delta = [
            -(v/w) * sin(theta) + (v/w) * sin(theta + w*dt);
             (v/w) * cos(theta) - (v/w) * cos(theta + w*dt);
             w * dt
        ];
        
        Delta = [
            0, 0, -(v/w) * cos(theta) + (v/w) * cos(theta + w*dt);
            0, 0, -(v/w) * sin(theta) + (v/w) * sin(theta + w*dt);
            0, 0, 0
        ];
    end
        
    % Intermediate matrices
    I = eye(3);
    Psi = Fx' * ((I + Delta)^(-1) - I) * Fx; % Info change due to motion
    lambda = Psi' * omega_prev + omega_prev * Psi + Psi' * omega_prev * Psi; % how much new info 
    Phi = omega_prev + lambda; % Pred info matrix
    kappa = Phi * Fx' / (Rt + Fx * Phi * Fx') * Fx * Phi; % Account for motion noise

    % omega update (Information matrix)
    % how confident in state estimates
    omega = Phi - kappa;

    % xi update (Information vector)
    xi = xi_prev + (lambda - kappa) * mu_prev + omega * Fx' * delta;

    % mu update (mean of state estimate)
    mu = mu_prev + Fx' * delta;
end