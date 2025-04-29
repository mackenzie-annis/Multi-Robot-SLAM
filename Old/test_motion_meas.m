% Load binary map and landmarks
load binary_map.mat  % gives `map` and `landmarks` (struct array)

% Robot initial true pose
x_true = [100; 100; pi/4];  % [x, y, theta]

% Motion params
u = [3; 0.1];            % control input
dt = 1.0;
alpha = [0.5, 0.1];      % motion noise std: [v_noise, w_noise]
robot_path = [];  


% Measurement params
max_range = 60;           % pixels
sigma_r = 1;              % range noise
sigma_phi = deg2rad(2);   % angle noise

% Initialize measured occupancy map (gray = unknown)
occ_map = 0.5 * ones(size(map));
alpha_occ = 0.6;  % smoothing factor (0.5–0.9 recommended)

% Convert landmark struct to Nx3 matrix: [x, y, id]
landmark_array = zeros(length(landmarks), 3);

for i = 1:length(landmarks)
    landmark_array(i, :) = [landmarks(i).x, landmarks(i).y, landmarks(i).id];
end

% TESTING SIMULATE FEATURE MEASUREMENTS & NOISY LIDAR SCAN
figure(1);
for t = 1:60
    % 1. Simulate motion
    x_true = apply_noisy_motion(x_true, u, dt, alpha);
    robot_path(end+1, :) = x_true(1:2)'; % for viz

    % 2. Update occupancy map with current scan
    occ_map = update_occupancy_map_lpf(occ_map, x_true, map, max_range, 1.0, alpha_occ);

    % 3. Simulate landmark measurements
    Z = simulate_feature_measurements(x_true, landmark_array, max_range, sigma_r, sigma_phi, map);

    % 4. Plot robot + landmarks
    figure(1); clf;
    imshow(map, 'InitialMagnification', 400); hold on;
    plot(landmark_array(:,1), landmark_array(:,2), 'ro');
    plot(x_true(1), x_true(2), 'bo', 'MarkerFaceColor', 'b');
    plot(robot_path(:,1), robot_path(:,2), 'b-', 'LineWidth', 1.5);

    for i = 1:size(Z, 1)
        r = Z(i,1); phi = Z(i,2);
        lx = x_true(1) + r * cos(x_true(3) + phi);
        ly = x_true(2) + r * sin(x_true(3) + phi);
        plot([x_true(1), lx], [x_true(2), ly], 'g--');
        plot(lx, ly, 'gx');
    end

    title(['Robot View - Step ', num2str(t)]);
    axis equal; drawnow;

    % 5. Plot occupancy map
    figure(2); clf;
    imshow(occ_map, 'InitialMagnification', 400);
    colormap(gray); title('Perceived Occupancy Map');
    drawnow;

    pause(0.3);
end

