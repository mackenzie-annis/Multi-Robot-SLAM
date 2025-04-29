function plotExplorationStepTwoRobots(robots, goals, map, landmarks, prob_maps, step_idx)
% Visualizes SLAM, occupancy, frontiers, and A* path for two robots.

    figure(1); clf;
    set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Maximize window

    for r = 1:2
        robot = robots{r};
        goal = goals{r};
        prob_map = prob_maps{r};
        slam = robot.slam;

        row_offset = (r-1)*2;

        % Subplot 1: EKF SLAM
        subplot(2, 2, 1 + row_offset);
        imshow(map); axis image; axis tight; hold on;
        title(sprintf('Robot %d: EKF SLAM (Step %d)', r, step_idx));

        % Landmarks
        plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
        for i = 1:size(landmarks,1)
            text(landmarks(i,1)+5, landmarks(i,2), num2str(landmarks(i,3)), ...
                'Color', 'c', 'FontSize', 8);
        end

        % True + estimated poses
        true_pose = robot.pose;
        est_pose = slam.mu(1:3);
        plot(true_pose(1), true_pose(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
        plot(est_pose(1), est_pose(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);

        % Pose shapes
        [true_circle, true_head] = robot.computePoseShape();
        plot(true_circle(1,:), true_circle(2,:), 'b');
        plot(true_head(1,:), true_head(2,:), 'b-', 'LineWidth', 2);
        offset = est_pose(1:2) - true_pose(1:2);
        [est_circle, est_head] = robot.computePoseShape();
        plot(est_circle(1,:) + offset(1), est_circle(2,:) + offset(2), 'g');
        plot(est_head(1,:) + offset(1), est_head(2,:) + offset(2), 'g-', 'LineWidth', 2);

        % Goal marker
        if ~isempty(goal)
            plot(goal(2), goal(1), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end

        % Uncertainty + landmark lines
        draw_ellipse(est_pose(1:2), slam.Sigma(1:2,1:2), 'b');
        
        % Plot measurement lines only for currently measured landmarks (in global coordinates)
        active_pts = robot.sensor.last_measurements;
        for k = 1:size(active_pts, 2)  % Iterate over each measured landmark
            mu_j = active_pts(:, k);  % Global coordinates of the measured landmark
            plot([est_pose(1), mu_j(1)], [est_pose(2), mu_j(2)], 'm--', 'LineWidth', 1.5);
        end


        legend('Landmarks', 'True Pose', 'EKF Pose', 'Goal', 'Location', 'bestoutside');

        % Subplot 2: Occupadancy Map + Frontiers + Path 
        subplot(2, 2, 2 + row_offset);
        displayProbabilityMap(prob_map, [2, 2, 2 + row_offset]);
        title(sprintf('Robot %d: Occupancy Map + Frontiers', r)); hold on;

        % Frontiers (green)
        frontiers = robot.frontiers;
        for i = 1:length(frontiers)
            pts = frontiers{i};
            scatter(pts(:,2), pts(:,1), 10, 'g', 'filled');
        end

        % Centroids (magenta)
        centroids = robot.centroids;
        if ~isempty(centroids)
            scatter(centroids(:,2), centroids(:,1), 60, 'm', 'filled');
        end

        % Goal (red X)
        if ~isempty(goal)
            scatter(goal(2), goal(1), 100, 'rx', 'LineWidth', 2);
        end

        % A* Path (green line)
        if ~isempty(robot.path)
            path = robot.path;
            plot(path(:,2), path(:,1), 'g-', 'LineWidth', 2);
        end
    end
end

