function path = astar(inflated_map, collision_map, start, goal)
    % ASTAR_DUAL_MAPS: A* path planning using inflated and strict collision maps
    % Inputs:
    %   inflated_map  - binary map (1 = inflated obstacle region, 0 = free)
    %   collision_map - binary map (1 = hard obstacle, 0 = free)
    %   start, goal   - [y, x] indices
    % Output:
    %   path - Nx2 array of [y, x] path coordinates

    [H, W] = size(inflated_map);

    % Heuristic: Euclidean distance
    heuristic = @(a, b) norm(a - b);

    % Priority queue setup
    frontier = PriorityQueue();
    frontier.insert(start, 0);

    came_from = containers.Map();
    cost_so_far = containers.Map();
    key = @(pt) sprintf('%d,%d', pt(1), pt(2));
    came_from(key(start)) = [];
    cost_so_far(key(start)) = 0;

    while ~frontier.isEmpty()
        current = frontier.pop();
        ckey = key(current);

        if isequal(current, goal)
            break;
        end

        % 8-connected neighbors
        for dy = -1:1
            for dx = -1:1
                if dy == 0 && dx == 0, continue; end
                ny = current(1) + dy;
                nx = current(2) + dx;
                if ny < 1 || ny > H || nx < 1 || nx > W, continue; end
        
                if collision_map(ny, nx) == 1
                    continue;  % hard obstacle
                end
        
                % Inflate zone logic
                entering_inflated = (inflated_map(ny, nx) == 1 && inflated_map(current(1), current(2)) == 0);
                deepening_inflated = (inflated_map(ny, nx) == 1 && inflated_map(current(1), current(2)) == 1);
                
        
                step_cost = norm([dy, dx]);
                istep_cost = norm([dy, dx]);

                % Add penalty for entering or staying inside inflated region
                if entering_inflated
                    step_cost = step_cost + 1;   % small penalty for entering inflated zone
                elseif deepening_inflated
                    step_cost = step_cost + 10;    % larger penalty for staying inside
                end

        
                next = [ny, nx];
                nkey = key(next);
                new_cost = cost_so_far(ckey) + step_cost;
                if ~isKey(cost_so_far, nkey) || new_cost < cost_so_far(nkey)
                    cost_so_far(nkey) = new_cost;
                    priority = new_cost + heuristic(next, goal);
                    frontier.insert(next, priority);
                    came_from(nkey) = current;
                end
            end
        end
    end

    % Reconstruct path
    path = [];
    curr = goal;
    while ~isempty(curr)
        path = [curr; path];
        curr_key = key(curr);
        if isKey(came_from, curr_key)
            curr = came_from(curr_key);
        else
            break;
        end
    end
end

