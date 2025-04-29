clear; clc;

% Load map and landmark data
load binary_map.mat  % provides `map` and `landmarks` struct

% Convert landmarks to Nx3 [x y id]
landmark_array = zeros(length(landmarks), 3);
for i = 1:length(landmarks)
    landmark_array(i, :) = [landmarks(i).x, landmarks(i).y, landmarks(i).id];
end

% ------------------
% Parameters
% ------------------
N = length(landmark_array);   % number of landmarks
dt = 1.0;
u = [3; 0.05];                % constant control input
alpha_motion = [0.5, 0.1];    % [v_noise, w_noise]
Rt = diag([1^2, 1^2, deg2rad(10)^2]);  % motion noise (covariance)

% Measurement noise
sigma_r = 0.5;
sigma_phi = deg2rad(1);
sigma_s = 0.01;

% LIDAR update map
occ_map = 0.5 * ones(size(map));
sigma_pixel = 0.5;
alpha_map = 0.6;

% ------------------
% Initialization
% ------------------
x_true = [30; 30; 0];     % initial true pose
x_true_path = [];
mu_before_path = [];
mu_after_path = [];
robot_path = [];

mu = [x_true; zeros(3*N, 1)];
xi = zeros(size(mu));
omega = eye(length(mu)) * 1e-3;
seen_landmarks = false(N,1);

% ------------------
% Main SEIF Loop
% ------------------

for t = 1:50
    % 1. True motion
    x_true = apply_noisy_motion(x_true, u, dt, alpha_motion);
    x_true_path(end+1, :) = x_true(1:2)';

    % 2. SEIF motion update
    [xi, omega, mu] = SEIF_motion_update(xi, omega, mu, u, Rt, dt, N);
    % 3. Simulate feature measurements
    Z = simulate_feature_measurements(x_true, landmark_array, 60, sigma_r, sigma_phi, map);

    if(~isempty(Z))
        ct = round(Z(:,3));  % known data association (landmark ID)
    end

    % 4. SEIF measurement update
    [xi, omega, seen_landmarks] = SEIF_measurement_update(xi, omega, mu, Z, ct, sigma_r, sigma_phi, sigma_s, seen_landmarks);

    % 5. Update estimate
    active_ids = find(seen_landmarks);
    %mu = SEIF_update_state_estimate(xi, omega, mu, active_ids);

    mu_after_path(end+1, :) = mu(1:2)';

    % 6. Update occupancy map from true pose (for comparison)
    occ_map = update_occupancy_map_lpf(occ_map, x_true, map, 60, sigma_pixel, alpha_map);

    % 7. Plot 
    figure(1); clf;
    imshow(map, 'InitialMagnification', 400); hold on;
    plot(landmark_array(:,1), landmark_array(:,2), 'ro'); % landmarks
    for i = 1:size(landmark_array, 1)
        text(landmark_array(i,1) + 3, landmark_array(i,2), ...
            num2str(landmark_array(i,3)), ...
            'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
    end


    plot(x_true_path(:,1), x_true_path(:,2), 'k-', 'LineWidth', 1.5);

    plot(mu_after_path(:,1), mu_after_path(:,2), 'b-', 'LineWidth', 1.5);
    plot(x_true(1), x_true(2), 'ko', 'MarkerFaceColor', 'k');
    plot(mu_after_path(end,1), mu_after_path(end,2), 'bo');

    % Plot landmark observation lines
    if ~isempty(Z)
        for i = 1:size(Z,1)
            r = Z(i,1);
            phi = Z(i,2);
            lx = x_true(1) + r * cos(x_true(3) + phi);
            ly = x_true(2) + r * sin(x_true(3) + phi);
            plot([x_true(1), lx], [x_true(2), ly], 'g--', 'LineWidth', 1.0);
            plot(lx, ly, 'gx');
        end
    end
   

    title(['SEIF Trajectory - Step ', num2str(t)]);
    legend('Landmarks', 'True Path', 'SEIF Corrected');
    axis equal; drawnow;


    pause(0.3);
end

