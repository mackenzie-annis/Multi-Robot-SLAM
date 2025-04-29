addpath('Helper'); 
clear; clc; close all;

% Load map and landmarks
data = load('Map/binary_map.mat');
map = data.map;
landmarks = data.landmarks;
landmarks = [landmarks, (1:size(landmarks,1))'];

% Initialize Sensor and Robot
sensor = Sensor('range', 200, 'sigma_r', 1.0, 'sigma_phi', deg2rad(1), 'angular_res', deg2rad(0.1));
robot = Robot([150; 150; pi/2], 1.0);
pose_hist = robot.getPose();

% Initialize occupancy map (log-odds)
log_odds_map = zeros(size(map));

% Setup figure with 2 subplots
figure;
subplot(1,2,1);
h_map = imshow(map); hold on;
title('Map with Robot and Hits');
h_occ_map = subplot(1,2,2);
h_occ_img = imagesc(flipud(map)); colormap(h_occ_map, gray); axis image;
title('Occupancy Grid'); hold on;

for t = 1:50
    % Move the robot
    robot.setControl([5; deg2rad(10)]);
    pose = robot.applyNoisyMotion(map);
    pose_hist = [pose_hist, pose];

    % LIDAR + landmarks
    [ranges, bearings] = sensor.lidarScan(map, pose);
    valid = ~isnan(ranges) & ranges < sensor.range;
    [x_hits, y_hits] = sensor.localToGlobalPolar(pose, ranges(valid), bearings(valid));
    [meas_pts, Z] = sensor.getLandmarkMeasurements(pose, landmarks, map);

    % Update occupancy map
    log_odds_map = occupancyMapping(log_odds_map, pose, sensor, map);
    prob_map = 1 - 1 ./ (1 + exp(log_odds_map));

    % ==== PLOT ====
    subplot(1,2,1); cla;
    imshow(map); hold on;
    plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    plot(pose_hist(1,:), pose_hist(2,:), 'b');  % path
    plot(x_hits, y_hits, 'rx', 'MarkerSize', 6, 'LineWidth', 1.2);
    [circle_pts, heading_line] = robot.computePoseShape();
    plot(circle_pts(1,:), circle_pts(2,:), 'b', 'LineWidth', 1.5);
    plot(heading_line(1,:), heading_line(2,:), 'b', 'LineWidth', 2);

    for i = 1:size(meas_pts, 2)
        plot([pose(1), meas_pts(1,i)], [pose(2), meas_pts(2,i)], 'c--');
    end
    title('Robot Map with Lidar Scans');

    % Plot occupancy grid
    displayOccupancyMap(log_odds_map, [1, 2, 2]);
    pause(0.2);
end

