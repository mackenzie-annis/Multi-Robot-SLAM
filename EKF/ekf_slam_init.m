function [mu, Sigma, observed_landmarks] = ekf_slam_init(num_landmarks, initial_pose)
% EKF_SLAM_INIT Initializes state and covariance for EKF SLAM
%   Inputs:
%       num_landmarks     - total number of landmarks
%       initial_pose      - [x; y; theta] robot pose
%   Outputs:
%       mu                - initial state vector [x; y; theta; l1x; l1y; l2x; l2y; ...]
%       Sigma             - initial covariance matrix
%       observed_landmarks- logical array tracking if a landmark has been seen

    % State vector 
    % Start with robot pose
    mu = zeros(3 + 2 * num_landmarks, 1);
    mu(1:3) = initial_pose;

    % --- Covariance matrix ---
    % Initial uncertainty: robot pose = small, landmarks = large
    Sigma = eye(length(mu)) * 1e6;  % Large uncertainty for all
    %Sigma(1:3,1:3) = 1e-3 * eye(3); % Small uncertainty for robot pose
    Sigma(1:3,1:3) = diag([1, 1, deg2rad(10)].^2);


    % --- Landmark initialization tracker ---
    observed_landmarks = false(1, num_landmarks);
end

