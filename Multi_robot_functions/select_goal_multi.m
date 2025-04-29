function [goal, frontiers, all_centroids] = select_goal(prob_map, last_goal, robot_pos, tolerance, other_goals)
    % SELECT_GOAL chooses a frontier to explore based on visibility, distance,
    % and multi-robot coordination.
    %
    % Inputs:
    %   prob_map: occupancy probability map
    %   last_goal: last chosen goal to prevent oscillation
    %   robot_pos: current [y, x] of this robot
    %   tolerance: distance threshold for goal reuse
    %   other_goals: [M x 2] array of other robots' goal centroids
    %
    % Outputs:
    %   goal: [y, x] selected frontier point
    %   frontiers: clustered frontiers
    %   all_centroids: full list of centroids

    robot_pos = robot_pos(:)'; % row vector
    frontiers = find_frontiers(prob_map, 6, robot_pos);
    centroids = [];  % visible
    centroids2 = []; % non-visible
    all_centroids = [];

    if isempty(frontiers)
        disp('EXPLORATION DONE');
        goal = last_goal;
        return;
    end

    for i = 1:length(frontiers)
        pts = frontiers{i};
        c = round(mean(pts, 1));
        if isVisible(prob_map, robot_pos(1), robot_pos(2), c(1), c(2))
            centroids = [centroids; c];
        else
            centroids2 = [centroids2; c];
        end
    end

    all_centroids = [centroids; centroids2];

    % Prefer to keep last goal if still close to a centroid
    if ~isempty(last_goal)
        dists_to_last = vecnorm(all_centroids - last_goal, 2, 2);
        close_idx = find(dists_to_last < tolerance);
        if ~isempty(close_idx)
            [~, subidx] = min(dists_to_last(close_idx));
            goal = round(all_centroids(close_idx(subidx), :));
            return;
        end
    end

    % Penalize centroids close to other robots' goals
    penalty = zeros(size(all_centroids, 1), 1);
    if ~isempty(other_goals)
        for i = 1:size(all_centroids, 1)
            for j = 1:size(other_goals, 1)
                d = norm(all_centroids(i,:) - other_goals(j,:));
                if d < 80 %check if within 80 pixels euclidan dist 20 scale factor really prioritize going to other landmarks 
                    penalty(i) = penalty(i) + 20*(800 - d);
                end
            end
        end
    end

    % Select visible if possible, else non-visible
    if ~isempty(centroids)
        dists = vecnorm(centroids - robot_pos, 2, 2);
        cost = dists + penalty(1:size(centroids, 1)); % adds penalty
        [~, idx] = min(cost);
        goal = round(centroids(idx, :));
    else
        dists = vecnorm(centroids2 - robot_pos, 2, 2);
        cost = dists + penalty(size(centroids, 1)+1:end);
        [~, idx] = min(cost);
        goal = round(centroids2(idx, :));
    end
end

function visible = isVisible(map, y1, x1, y2, x2)
    pts = Sensor.bresenham(x1, y1, x2, y2); % [x, y]
    visible = true;
    for i = 1:size(pts,1)
        x = pts(i,1);
        y = pts(i,2);
        if map(y, x) == 1
            visible = false;
            return;
        end
    end
end
