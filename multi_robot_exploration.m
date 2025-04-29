clear; clc; close all;
addpath(genpath('~/Documents/MATLAB/autonomous_robots'));

% CONFIGURATION
config = configuration();
robots = config.robots;
map = config.map;
landmarks = config.landmarks;
num_robots = config.num_robots;
max_steps = config.max_steps;
uncertainty_thresh = 500;

last_goals = cell(1, num_robots);

% MAIN LOOP
for t = 1:20
    % Update and Move Each Robot
    for i = 1:num_robots
        robot = robots{i};
        robot.updateMapAndSLAM(landmarks, map);

        prob_map = 1 - 1 ./ (1 + exp(robot.log_odds_map));
        prob_map = inflateProbMap(prob_map, 3);

        % Goal Selection
        robot_rc = poseToRowCol(robot.pose);
        [goal, frontiers, centroids] = select_goal_multi(prob_map, last_goals{i}, robot_rc, 10, ...
                                                         getOtherGoals(robots, i));
        last_goals{i} = goal;
        robot.goal = goal;
        robot.frontiers = frontiers;
        robot.centroids = centroids;

        % A* Path Planning
        inflated_map = inflateProbMap(prob_map, config.astar.inflation_radius);
        path = astar(inflated_map, prob_map > 0.5, robot_rc, goal);
        robot.path = path;

        % Move One Step Along Path 
        if size(path, 1) >= 2
            next = path(2, :);
            dx = next(2) - robot_rc(2);
            dy = next(1) - robot_rc(1);
            angle_to_next = atan2(dy, dx);
            theta = robot.pose(3);
            robot.setControl([5; angle_to_next - theta]);
            robot.applyNoisyMotion(map);
        end
    end

    % Plot All Robots
    goals = cellfun(@(r) r.goal, robots, 'UniformOutput', false);
    prob_maps = cellfun(@(r) 1 - 1 ./ (1 + exp(r.log_odds_map)), robots, 'UniformOutput', false);
    plotExplorationStepTwoRobots(robots, goals, map, landmarks, prob_maps, t);

    % Landmark Fusion (If Robots Are Close)
    for i = 1:num_robots
        for j = i+1:num_robots % avoids duplicate paqiring 
            pos_i = robots{i}.slam.mu(1:2); % pos 1
            pos_j = robots{j}.slam.mu(1:2); % pos 2


            Fusion_triggered = false;
            if norm(pos_i - pos_j) < 100  % Fusion threshold
                Fusion_triggered = true;
                fprintf('[Step %d] Fusion triggered between Robot %d and Robot %d\n', t, i, j);
    
                % % Landmark Fusion
                % for l = 1:size(landmarks, 1)
                %     [fused_mu, fused_Sigma, updated_seen] = fuse_landmark_estimates(...
                %         robots{i}.slam.mu, robots{i}.slam.Sigma, ...
                %         robots{j}.slam.mu, robots{j}.slam.Sigma, ...
                %         robots{i}.slam.seen_landmarks, robots{j}.slam.seen_landmarks, l);
                % 
                %     if ~isempty(fused_mu)
                %         idx = 3 + 2*(l-1) + 1;
                %         for r = [i, j]
                %             robots{r}.slam.mu(idx:idx+1) = fused_mu;
                %             robots{r}.slam.Sigma(idx:idx+1, idx:idx+1) = fused_Sigma;
                %             robots{i}.slam.seen_landmarks(l) = updated_seen;
                %         end
                %     end
                % end
                

                % Occupancy Map Fusion (log-odds addition)
                eps = 0.01;
                map1 = robots{i}.log_odds_map;
                map2 = robots{j}.log_odds_map;
                
                explored1 = abs(map1) > eps;
                explored2 = abs(map2) > eps;
                
                both = explored1 & explored2;
                only1 = explored1 & ~explored2;
                only2 = explored2 & ~explored1;
                
                fused_map = zeros(size(map1));
                fused_map(both) = map1(both) + map2(both);
                fused_map(only1) = map1(only1);
                fused_map(only2) = map2(only2);
                fused_map = max(min(fused_map, 10), -10);  % optional clamp
                
                for r = [i, j]
                    robots{r}.log_odds_map = fused_map;
                end
            end
        end
    end

    % === Subplot 1: Occupancy Grid Map ===
    figure(99); clf;
    subplot(1,2,1);
    log_odds_map = robots{1}.log_odds_map;  % assuming shared fused map
    displayOccupancyMap(log_odds_map, [1,2,1]);
    title('Fused Occupancy Map');
    
    % === Subplot 2: Fused Landmark Positions ===
    subplot(1,2,2); hold on; axis equal;
    colors = lines(length(robots));
    
    for r = 1:length(robots)
        mu = robots{r}.slam.mu;
        seen = robots{r}.slam.seen_landmarks;
    
        for l = find(seen)
            idx = 3 + 2*(l-1) + 1;
            pos = mu(idx:idx+1);
            plot(pos(1), pos(2), 'o', 'MarkerSize', 8, ...
                'MarkerFaceColor', colors(r,:), 'DisplayName', sprintf('Robot %d - LM %d', r, l));
        end
    end

end

disp('Exploration Complete');

function other_goals = getOtherGoals(robots, self_idx)
    other_goals = [];
    for k = 1:length(robots)
        if k ~= self_idx && ~isempty(robots{k}.goal)
            other_goals = [other_goals; robots{k}.goal];
        end
    end
end
