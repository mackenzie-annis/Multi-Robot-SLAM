function occ_map = update_occupancy_map_lpf(occ_map, x_true, map, max_range, sigma_pixel, alpha)
% Updates the occupancy map calling noisy_lidat_scan and average smoothing 
%   occ_map     - current occupancy grid  where 1 = free, 0.5 = unknown, 0 = occupied
%   x_true  - [x; y; theta] true robot pose from noisy motion
%   map         - binary occupancy ground truth (0 = wall, 1 = free)
%   max_range   - max LIDAR scan distance (pixels)
%   p_hit       - prob of correctly seeing obstacle
%   p_false_pos - prob of hallucinating obstacle
%   alpha       - smoothing factor (e.g., 0.6)

    % Get current LIDAR scan
    [obs_occupied, obs_free] = noisy_lidar_scan(map, x_true, max_range, sigma_pixel);

    [H, W] = size(occ_map);

    % Update occupied cells (toward 1.0 = black)
    for i = 1:size(obs_occupied, 1)
        x = obs_occupied(i,1);
        y = obs_occupied(i,2);
        if x >= 1 && x <= W && y >= 1 && y <= H
            occ_map(y, x) = alpha * 0.0 + (1 - alpha) * occ_map(y, x);
        end
    end

    % Update free cells (toward 0.0 = white)
    for i = 1:size(obs_free, 1)
        x = obs_free(i,1);
        y = obs_free(i,2);
        if x >= 1 && x <= W && y >= 1 && y <= H
            occ_map(y, x) = alpha * 1.0 + (1 - alpha) * occ_map(y, x);
        end
    end
end
