function Z = simulate_feature_measurements(xtrue, landmarks, max_range, sigma_r, sigma_phi, map)
    % Inputs:
    % xtrue: [x; y; theta] robot pose
    % landmarks: Nx3 [x, y, signature]
    % max_range: visibility radius
    % sigma_r, sigma_phi: noise std devs
    % map: binary occupancy map (1 = free, 0 = wall)

    Z = [];  % Each row: [range, bearing, signature]

    for i = 1:size(landmarks, 1)
        lx = landmarks(i,1);
        ly = landmarks(i,2);
        sig = landmarks(i,3);

        dx = lx - xtrue(1);
        dy = ly - xtrue(2);
        r = sqrt(dx^2 + dy^2);
        phi = wrapToPi(atan2(dy, dx) - xtrue(3));

        if r <= max_range
            % Check visibility
            if is_visible(map, xtrue(1), xtrue(2), lx, ly)
                % Add noise
                r_noisy = r + randn() * sigma_r;
                phi_noisy = wrapToPi(phi + randn() * sigma_phi);
                Z(end+1, :) = [r_noisy, phi_noisy, sig];
            end
        end
    end
end


%% Helper Functions
% Want to check before accepting measurement whether the robot to landmark
% line of site passes through a wall to simulate a lidar measurement 
function visible = is_visible(map, x1, y1, x2, y2)
    line_points = bresenham_line(round(x1), round(y1), round(x2), round(y2));
    visible = true;
    for k = 1:size(line_points, 1)
        x = line_points(k,1);
        y = line_points(k,2);
        if x < 1 || y < 1 || y > size(map,1) || x > size(map,2)
            visible = false;
            return;
        end
        if map(y, x) == 0  % hit wall
            visible = false;
            return;
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
