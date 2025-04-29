addpath(genpath('/Users/mackenzie/Documents/MATLAB/autonomous_robots/'));

clear; clc; close all;

% === Load Map and Landmarks ===
data = load('Map/binary_map.mat');
map = data.map;
landmarks = data.landmarks;
landmarks = [landmarks, (1:size(landmarks,1))'];  % Add IDs

% === Sensor and Robot Setup ===
sensor = Sensor('range', 100, 'sigma_r', 10.0, 'sigma_phi', deg2rad(5), 'angular_res', deg2rad(2));
dt = 1.0;
robot_true = Robot([150; 150; pi/2], dt, [0.5; deg2rad(5)]);  % motion noise

% === EKF Initialization ===
N = size(landmarks, 1);  % Number of landmarks
[mu, Sigma, seen_landmarks] = ekf_slam_init(N, robot_true.getPose());
%Rt = diag([0.5^2, deg2rad(5)^2, deg2rad(5)^2]);  % motion noise covariance
Rt = diag([1.0^2, 1.0)^2, deg2rad(5)^2]);


% === Storage ===
true_path = robot_true.getPose();
est_path = mu(1:3);

% Visualization Setup
figure;
h_map = imshow(map); hold on;
title('True Pose vs EKF Estimate');

% === Simulation Loop ===
for t = 1:20
    % --- Control and True Motion ---
    u = [5; deg2rad(10)];
    robot_true.setControl(u);
    pose = robot_true.applyNoisyMotion(map);

    % --- EKF Motion Update ---
    [mu, Sigma] = ekf_motion_update(mu, Sigma, u, Rt, dt, N);

    % --- Landmark Measurements ---
    Z = sensor.measureLandmarks(pose, landmarks, map);

    % --- EKF Measurement Update ---
    [mu, Sigma, seen_landmarks] = ekf_slam_update(mu, Sigma, Z, sensor.sigma_r, sensor.sigma_phi, seen_landmarks);

    % --- Log Paths ---
    true_path = [true_path, pose];
    est_path = [est_path, mu(1:3)];

    % --- Visualization ---
    imshow(map); hold on;
    plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    for i = 1:size(landmarks,1)
        text(landmarks(i,1)+5, landmarks(i,2), num2str(landmarks(i,3)), 'Color', 'red', 'FontSize', 8);
    end

    plot(true_path(1,:), true_path(2,:), 'b-', 'LineWidth', 2);
    plot(est_path(1,:), est_path(2,:), 'g--', 'LineWidth', 2);

    % Draw robot shapes
    [circle_true, head_true] = robot_true.computePoseShape();
    plot(circle_true(1,:), circle_true(2,:), 'b');
    plot(head_true(1,:), head_true(2,:), 'b', 'LineWidth', 2);

    [circle_est, head_est] = robot_true.computePoseShape();
    circle_est = circle_est + (mu(1:2) - pose(1:2));
    head_est = head_est + (mu(1:2) - pose(1:2));
    plot(circle_est(1,:), circle_est(2,:), 'g');
    plot(head_est(1,:), head_est(2,:), 'g', 'LineWidth', 2);
    
    plot(est_path(1,:), est_path(2,:), 'g--', 'LineWidth', 2);
    % === Draw lines from robot to seen landmarks ===
    robot_pos = mu(1:2);
    
    for i = 1:size(Z, 1)
        landmark_id = Z(i, 3);  % ID = s = j
        ldx = mu(3 + 2*(landmark_id-1) + 1);
        ldy = mu(3 + 2*(landmark_id-1) + 2);
    
        plot([robot_pos(1), ldx], [robot_pos(2), ldy], 'm--', 'LineWidth', 1.5);
    end

    % Draw robot pose uncertainty
    draw_ellipse(mu(1:2), Sigma(1:2,1:2), 'b');
    
    % Draw ellipses for each seen landmark
    for j = 1:length(seen_landmarks)
        if seen_landmarks(j)
            idx = 3 + 2*(j-1) + 1;
            l_mu = mu(idx:idx+1);
            l_cov = Sigma(idx:idx+1, idx:idx+1);
            draw_ellipse(l_mu, l_cov, 'c');
        end
    end

    legend('Landmarks', 'True Path', 'SLAM Estimate', 'Location', 'SouthOutside');
    title(sprintf('SLAM vs Ground Truth (Step %d)', t));
    drawnow;
    pause(0.5)

end
