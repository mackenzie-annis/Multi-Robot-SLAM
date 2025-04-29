% Load map and landmarks
data = load('Map/binary_map.mat');
map = data.map;
log_odds_map = zeros(size(map));

% Init robot and sensor
sensor = Sensor('range', 150, 'sigma_r', 1.0, 'sigma_phi', deg2rad(1), 'angular_res', deg2rad(0.1));
robot = Robot();
pose_hist = robot.pose;
last_goal = [];

% Exploration loop
for t = 1:1000
    % Step 1: Sense
    pose = robot.pose;
    log_odds_map = occupancyMapping(log_odds_map, pose, sensor, map);
    prob_map = 1 - 1 ./ (1 + exp(log_odds_map));
    prob_map = inflateProbMap(prob_map,3);
    prob_map_inflated = inflateProbMap(prob_map, 15); 
    figure(1)
    imagesc(prob_map)
     
    % Step 2: Find frontiers
        robot_pos = round(flip(pose(1:2)));
        frontiers = find_frontiers(prob_map, 6, robot_pos');
    
        centroids = []; % visible centroids (Priority)
        centroids2 = []; % Centroids the robot knows are on the map
        if ~isempty(frontiers)
            pose = robot.pose;            
            robot_pos = round(flip(pose(1:2)));
            
            for i = 1:length(frontiers)
                pts = frontiers{i};
                c = round(mean(pts, 1));  % centroid
                if isVisible(prob_map, robot_pos(1), robot_pos(2), c(1), c(2))
                    centroids = [centroids; c];  % append visible centroids only
                else
                    centroids2 = [centroids2; c];
                end
            end
            
            % Check if there are are any visible centroids 
            if centroids
                if isempty(last_goal)
                    % No previous goal, pick closest to robot
                    dists = vecnorm(centroids - robot_pos', 2, 2);
                    [~, idx] = min(dists);
                    goal = round(centroids(idx, :));
                else
                    tolerance = 10;
                    % Find centroids close to last_goal
                    dists_to_last = vecnorm(centroids - last_goal, 2, 2);
                    close_idx = find(dists_to_last < tolerance);
                
                    if ~isempty(close_idx)
                        % Pick the closest one to last_goal
                        [~, subidx] = min(dists_to_last(close_idx));
                        goal = round(centroids(close_idx(subidx), :));
                    else
                        % No centroid near last_goal → pick closest to robot
                        dists_to_robot = vecnorm(centroids - robot_pos', 2, 2);
                        [~, idx] = min(dists_to_robot);
                        goal = round(centroids(idx, :));
                    end
                end
            else % go to closest non visible centroid
                dists = vecnorm(centroids2 - robot_pos', 2, 2);
                [~, idx] = min(dists);
                goal = round(centroids2(idx, :));
            end
            last_goal = goal;
        else
            goal = last_goal;
            if isempty(goal), break; end
        end
    
    % Step 4: Plan path
    robot_pos = robot.pose;
    start = round(robot_pos(1:2));
    start = flip(start)';
    path = astar(prob_map_inflated, prob_map, start, goal);% TAKES START AND GOAL IN [y,x]
   
    % Step 5: Move one step along path
    if size(path, 1) >= 2
        next = path(2, :);
        dy = next(1) - start(1);
        dx = next(2) - start(2);
        angle_to_next = atan2(dy, dx);
        pose = robot.pose;
        theta = pose(3);
        w = angle_to_next - theta;
        v = 5;

        robot.setControl([v; w]);
        robot.applyNoisyMotion(map);
        pose_hist = [pose_hist, robot.pose];
    end

    % Visualization
    figure(2); clf;
    displayProbabilityMap(prob_map, [1,1,1]);
    hold on;
    [circle_pts, heading_line] = robot.computePoseShape();
    plot(circle_pts(1,:), circle_pts(2,:), 'b-', 'LineWidth', 1.5);
    plot(heading_line(1,:), heading_line(2,:), 'b-', 'LineWidth', 1.5);

    for i = 1:length(frontiers)
        pts = frontiers{i};
        scatter(pts(:,2), pts(:,1), 10, 'g', 'filled'); 
    end

    % Plot all centroids
    % Plot centroids and save handles
    % Clear previously plotted centroid markers if they exist
    centroid_handles = [];
    if exist('centroid_handles', 'var')
        delete(centroid_handles(:));
    end
    % reset
    if centroids
        centroid_handles(end+1) = scatter(centroids(:,2), centroids(:,1), 60, 'm', 'filled');
    end 
    
    scatter(goal(2), goal(1), 100, 'rx');
    if ~isempty(path)
        plot(path(:,2), path(:,1), 'g-', 'LineWidth', 2);
    end

    title('Frontier Exploration');
    drawnow;
end
disp('END')


% helper for brehesiman lin
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
