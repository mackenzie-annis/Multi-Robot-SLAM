function [obs_occupied, obs_free] = noisy_lidar_scan(map, robot_pose, max_range, sigma_pixel)
% Simplified LIDAR-style scan:
%   - Walls are always detected (no false negatives)
%   - Free space is correctly seen (no false positives)
%   - Wall measurements are noisy (slight pixel perturbation)

    [H, W] = size(map);
    cx = round(robot_pose(1));
    cy = round(robot_pose(2));

    obs_occupied = [];
    obs_free = [];

    for dx = -max_range:max_range
        for dy = -max_range:max_range
            dist = sqrt(dx^2 + dy^2);
            if dist <= max_range
                x = cx + dx;
                y = cy + dy;

                if x < 1 || x > W || y < 1 || y > H
                    continue
                end

                % Trace ray to (x, y)
                ray = bresenham_line(cx, cy, x, y);
                wall_count = 0;

                for k = 2:size(ray,1)  % skip robot cell
                    px = ray(k,1);
                    py = ray(k,2);

                    if px < 1 || px > W || py < 1 || py > H
                        break
                    end

                    true_val = map(py, px);  % 1 = free, 0 = wall

                    if true_val == 0  % wall
                        wall_count = wall_count + 1;
                        if wall_count > 3
                            break  % stop ray
                        end

                        % Add small pixel-level noise (±1)
                        px_n = px + round(randn() * sigma_pixel);
                        py_n = py + round(randn() * sigma_pixel);

                        % Clamp to bounds
                        px_n = min(max(px_n, 1), W);
                        py_n = min(max(py_n, 1), H);

                        obs_occupied(end+1, :) = [px_n, py_n];
                    else
                        % Free cell
                        obs_free(end+1, :) = [px, py];
                    end
                end
            end
        end
    end
end



function pts = bresenham_line(x0, y0, x1, y1)
    dx = abs(x1 - x0); dy = abs(y1 - y0);
    sx = sign(x1 - x0); sy = sign(y1 - y0);
    err = dx - dy; pts = [];
    while true
        pts(end+1,:) = [x0, y0];
        if x0 == x1 && y0 == y1, break; end
        e2 = 2 * err;
        if e2 > -dy, err = err - dy; x0 = x0 + sx; end
        if e2 < dx, err = err + dx; y0 = y0 + sy; end
    end
end

