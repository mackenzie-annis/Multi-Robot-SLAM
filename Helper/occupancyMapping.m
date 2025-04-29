function log_odds_map = occupancyMapping(log_odds_map, robot_pose, sensor, map)
% Updates the occupancy map using log-odds, based on LIDAR measurements
% from a Sensor object and Robot pose.
% Inputs:
%   log_odds_map - [HxW] current log-odds map
%   robot_pose   - [x; y; theta] robot position
%   sensor       - Sensor object (with lidarScan and localToGlobalPolar)
%   map          - binary occupancy map (1 = free, 0 = wall)

    [H, W] = size(log_odds_map);

    % Log-odds update values
    l_occ = 2.0;   % arbitrary positive value for occupancy
    l_free = -1.0; % negative value for free space
    l0 = 0;        % log odds of 0.5 prior

    % Get LIDAR measurements
    [ranges, bearings, ideal_ranges] = sensor.lidarScan(map, robot_pose);

    % Compute free-space cells (ray stepping up to hit)
    free_pts = [];
    for i = 1:length(ranges)
        r_ideal = ideal_ranges(i);  % use ideal ray distance
        phi = bearings(i);    
    
        r_vals = sensor.range_min : 1 : floor(r_ideal - 1);  %before hit
        phi_vals = repmat(phi, size(r_vals));
        [xf, yf] = sensor.localToGlobalPolar(robot_pose, r_vals, phi_vals);
        free_pts = [free_pts; round([xf(:), yf(:)])];
    end

    % Add a free cell at robot pose
    free_pts = [free_pts; round([robot_pose(1), robot_pose(2)])];


    % Compute occupied cells (endpoints)
    valid = ~isnan(ranges) & ranges < sensor.range;
    [x_occ, y_occ] = sensor.localToGlobalPolar(robot_pose, ranges(valid), bearings(valid));
    occupied_pts = round([x_occ(:), y_occ(:)]);

    % Update log-odds for occupied cells
    for i = 1:size(occupied_pts,1)
        x = occupied_pts(i,1); y = occupied_pts(i,2);
        if x >= 1 && x <= W && y >= 1 && y <= H
            log_odds_map(y, x) = log_odds_map(y, x) + (l_occ - l0);
        end
    end

    % Update log-odds for free cells
    for i = 1:size(free_pts,1)
        x = free_pts(i,1); y = free_pts(i,2);
        if x >= 1 && x <= W && y >= 1 && y <= H
            log_odds_map(y, x) = log_odds_map(y, x) + (l_free - l0);
        end
    end
end