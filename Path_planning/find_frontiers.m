function frontier_clusters = find_frontiers(prob_map, min_size, robot_pos)
    % Detects and clusters accessible frontier points from the robot position
    % Inputs:
    %   prob_map: [H x W] - 0 = free, 0.5 = unknown, 1 = occupied
    %   min_size: minimum cluster size
    %   robot_pos: [y, x] robot position to check reachability

    [H, W] = size(prob_map);
    frontier_map = false(H, W);

    % Step 1: Detect frontier points
    for y = 2:H-1
        for x = 2:W-1
            if prob_map(y,x) < 0.3  % free space
                neighbors = prob_map(y-1:y+1, x-1:x+1);
                if any(abs(neighbors(:) - 0.5) < 1e-6)  % has unknown neighbor
                    frontier_map(y,x) = true;
                end
            end
        end
    end

    % Step 2: Cluster frontier points
    conn = bwconncomp(frontier_map, 8);
    frontier_clusters = cell(1, 0);

    % Step 3: Build reachability mask using BFS
    reachable = false(H, W);
    visited = false(H, W);
    q = robot_pos;  % queue of [y, x]
    reachable(robot_pos(1), robot_pos(2)) = true;

    while ~isempty(q)
        p = q(1,:); q(1,:) = [];
        y = p(1); x = p(2);
        for dy = -1:1
            for dx = -1:1
                ny = y + dy; nx = x + dx;
                if ny >= 1 && ny <= H && nx >= 1 && nx <= W ...
                        && ~visited(ny, nx) && prob_map(ny, nx) == 0
                    visited(ny, nx) = true;
                    reachable(ny, nx) = true;
                    q(end+1, :) = [ny, nx];
                end
            end
        end
    end

    % Step 4: Keep only accessible frontier clusters
    for i = 1:conn.NumObjects
        inds = conn.PixelIdxList{i};
        [y, x] = ind2sub(size(prob_map), inds);
        cluster = [y(:), x(:)];

        if size(cluster, 1) >= min_size
            % Check if at least one point is reachable
            accessible = any(reachable(sub2ind(size(prob_map), y, x)));
            if accessible
                frontier_clusters{end+1} = cluster;
            end
        end
    end
end

