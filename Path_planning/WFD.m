function frontiers = WFD(prob_map, robot_pos)
    % Assumes prob_map where:
    %   0   = free
    %   0.5 = unknown
    %   1   = occupied
    % Input: prob_map (HxW matrix), robot_pos = [x, y]
    % Output: frontiers = cell array of frontier clusters

    % Map size
    [H, W] = size(prob_map);

    % Robot start position
    rx = round(robot_pos(1));
    ry = round(robot_pos(2));

    % State flags (for marking visited states)
    UNKNOWN = 0;
    MAP_OPEN = 1;
    MAP_CLOSED = 2;
    FRONTIER_OPEN = 3;
    FRONTIER_CLOSED = 4;

    % Initialize visited map - track which cells have been visited 
    visited = zeros(H, W);
    frontiers = {}; % Stores group of frontier cells

    % BFS queue from robot position
    queue_m = [ry, rx];
    visited(ry, rx) = MAP_OPEN;


    % BFS to find frontiers
    while ~isempty(queue_m)

        p = queue_m(1, :);
        queue_m(1, :) = [];
        py = p(1); px = p(2);

        if visited(py, px) == MAP_CLOSED
            continue;
        end

        if isFrontier(py, px, prob_map)
            queue_f = [py, px];
            frontier = [];
            visited(py, px) = FRONTIER_OPEN;
            
            % BFS for frontier Cluster
            while ~isempty(queue_f)
                q = queue_f(1, :);
                queue_f(1, :) = []; % Deque
                qy = q(1); qx = q(2);

                if visited(qy, qx) == FRONTIER_CLOSED || visited(qy, qx) == MAP_CLOSED % Skip if processd
                    continue;
                end
                
                if isFrontier(qy, qx, prob_map)
                    frontier = [frontier; qy, qx]; % Iteratively build frontier cluster
                    neighbors = getNeighbors(qy, qx, H, W);
                    for i = 1:size(neighbors, 1)
                        ny = neighbors(i, 1);
                        nx = neighbors(i, 2);
                        if visited(ny, nx) == UNKNOWN % Check if the neighbor of frontier have been visited 
                            queue_f = [queue_f; ny, nx]; % Unvisited add to queue
                            visited(ny, nx) = FRONTIER_OPEN;
                        end
                    end
                    visited(qy, qx) = FRONTIER_CLOSED; % Mark visited
                end
            end
            
            % After processing save frontier cluster
            if ~isempty(frontier)
                frontiers{end+1} = frontier;
                for i = 1:size(frontier, 1)
                    visited(frontier(i,1), frontier(i,2)) = MAP_CLOSED; % Mark as closed so not revisited
                end
            end
        end

        % Enqueue adjacent cells (Outer BFS to neighboring free cells) 
        neighbors = getNeighbors(py, px, H, W);
        for i = 1:size(neighbors, 1)
            ny = neighbors(i, 1);
            nx = neighbors(i, 2);
            if visited(ny, nx) == UNKNOWN && prob_map(ny, nx) == 0
                queue_m = [queue_m; ny, nx];
                visited(ny, nx) = MAP_OPEN; 
            end
        end
        visited(py, px) = MAP_CLOSED; % Once a cell is visited mark it as closed
    end
end

% connected neighbors
function nbs = getNeighbors(y, x, H, W)
% Returns 8 connected neighbor indices ignoring edge of a map 
    nbs = [];
    for dy = -1:1
        for dx = -1:1
            if dy == 0 && dx == 0
                continue;
            end
            ny = y + dy;
            nx = x + dx;
            if ny >= 2 && ny <= H-1 && nx >= 2 && nx <= W-1
                nbs = [nbs; ny, nx];
            end
        end
    end
end

% Helper function to check frontier condition
function is_frontier = isFrontier(y, x, prob_map)
    if prob_map(y, x) ~= 0
        is_frontier = false;
        return;
    end
    nhood = prob_map(y-1:y+1, x-1:x+1);
    is_frontier = any(nhood(:) == 0.5);
end