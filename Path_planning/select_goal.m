function [goal, frontiers, all_centroids] = select_goal(prob_map, last_goal, robot_pos, tolerance)
    % Given the probability_map selects the goal based on Frontier based exploration 
    % Inputs
    % prob_map: Probability map generated from log odds/occupancy mapping
    % last_goal 
    % robot_pos: robot position
    % tolerance: tolerance for checking if centroids are from the same
    % horizon as previous timestep
    
    %Find frontier clusters - dispose of clusters 
    robot_pos = robot_pos(:)'; % Ensure row vector
    frontiers = find_frontiers(prob_map, 6, robot_pos);
    centroids = [];  % visible (Priority)
    centroids2 = []; % non-visible
    all_centroids = [];
    % No frontiers
    if isempty(frontiers)
        disp('EXPLORATION DONE')
        goal = last_goal;
        return;
    end
    
    % Group frontiers into centroids visible from the robot and not
    % 
    for i = 1:length(frontiers)
        pts = frontiers{i};
        c = round(mean(pts, 1));
        if isVisible(prob_map, robot_pos(1), robot_pos(2), c(1), c(2))
            centroids = [centroids; c];
        else
            centroids2 = [centroids2; c];
        end
    end

    % total list 
    all_centroids = [centroids; centroids2];
    

    % To avoid oscillations the last centroid chosen as goal if avalable
    if ~isempty(last_goal)
        % See if last_goal exists within tolerance in etither list 
        
        dists_to_last = vecnorm(all_centroids - last_goal, 2, 2);
        close_idx = find(dists_to_last < tolerance);
        % Found a matching goal return
        if ~isempty(close_idx)
                [~, subidx] = min(dists_to_last(close_idx));
                goal = round(all_centroids(close_idx(subidx), :));
                return
        end
    end
    
    % Check the visible centroids first
    if ~isempty(centroids)
        dists = vecnorm(centroids - robot_pos, 2, 2);
        [~, idx] = min(dists);
        goal = round(centroids(idx, :));
    else
        dists = vecnorm(centroids2 - robot_pos, 2, 2);
        [~, idx] = min(dists);
        goal = round(centroids2(idx, :));
    end
    return 
end

% helper for brehesiman line
function visible = isVisible(map, y1, x1, y2, x2)
    pts = Sensor.bresenham(x1, y1, x2, y2); % [x, y] path
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