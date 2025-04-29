function path = astar(inflated_map, collision_map, start, goal)
    [H, W] = size(inflated_map);
    total_nodes = H * W;

    % Convert to linear index
    toIdx = @(pt) sub2ind([H W], pt(1), pt(2));
    fromIdx = @(idx) ind2sub_vec([H W], idx);

    % Heuristic
    heuristic = @(a, b) norm(a - b);

    % Cost & tracking
    cost = inf(H, W);
    parent = zeros(H, W);
    visited = false(H, W);

    cost(start(1), start(2)) = 0;
    goal_idx = toIdx(goal);

    % OPEN list: [f_cost, idx]
    open_list = [heuristic(start, goal), toIdx(start)];

    while ~isempty(open_list)
        % Pop lowest-cost node
        [~, i] = min(open_list(:,1));
        current_idx = open_list(i,2);
        open_list(i,:) = [];
        [cy, cx] = ind2sub([H, W], current_idx);

        if current_idx == goal_idx
            break;
        end

        if visited(cy, cx)
            continue;
        end
        visited(cy, cx) = true;

        for dy = -1:1
            for dx = -1:1
                if dy == 0 && dx == 0, continue; end
                ny = cy + dy;
                nx = cx + dx;
                if ny < 1 || ny > H || nx < 1 || nx > W, continue; end
                if collision_map(ny, nx) == 1, continue; end

                % Penalty logic
                step_cost = norm([dy dx]);
                if inflated_map(ny, nx) == 1 && inflated_map(cy, cx) == 0
                    step_cost = step_cost + 1;  % enter
                elseif inflated_map(ny, nx) == 1
                    step_cost = step_cost + 10; % stay in
                end

                new_cost = cost(cy, cx) + step_cost;
                if new_cost < cost(ny, nx)
                    cost(ny, nx) = new_cost;
                    parent(ny, nx) = current_idx;
                    f = new_cost + heuristic([ny nx], goal);
                    open_list(end+1,:) = [f, toIdx([ny nx])];
                end
            end
        end
    end

    % Reconstruct path
    path = [];
    curr = goal_idx;
    while curr ~= 0
        [y, x] = ind2sub([H W], curr);
        path = [path; y, x];
        curr = parent(y, x);
    end
    path = flipud(path);
end

function [y, x] = ind2sub_vec(sz, idx)
    y = mod(idx - 1, sz(1)) + 1;
    x = floor((idx - 1) / sz(1)) + 1;
end
