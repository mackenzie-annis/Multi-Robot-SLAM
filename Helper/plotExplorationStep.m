function plotExplorationStep(robot, goal, map, landmarks, prob_map, step_idx)
% plotExplorationStep - Visualize one timestep of SLAM + occupancy mapping
%
% Inputs:
%   robot      - Robot object with .pose and .slam properties
%   goal       - [y, x] coordinates of selected goal
%   map        - Ground truth map (binary)
%   landmarks  - [x, y, id] landmark positions
%   prob_map   - Current probability map
%   step_idx   - Current timestep

    figure(1); clf;
    set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Maximize window
    % Subplot 1: EKF SLAM Estimate
    subplot(1, 2, 1);
    imshow(map); axis image; axis tight; hold on;
    title(sprintf('EKF SLAM Estimate (Step %d)', step_idx));
    plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    for i = 1:size(landmarks,1)
        text(landmarks(i,1)+5, landmarks(i,2), num2str(landmarks(i,3)), 'Color', 'c', 'FontSize', 8);
    end

    % True and estimated poses
    true_pose = robot.pose;
    est_pose = robot.slam.mu(1:3);
    plot(true_pose(1), true_pose(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    plot(est_pose(1), est_pose(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);

    % Pose shapes
    [true_circle, true_head] = robot.computePoseShape();
    plot(true_circle(1,:), true_circle(2,:), 'b');
    plot(true_head(1,:), true_head(2,:), 'b-', 'LineWidth', 2);

    [est_circle, est_head] = robot.computePoseShape();
    offset = est_pose(1:2) - true_pose(1:2);
    plot(est_circle(1,:) + offset(1), est_circle(2,:) + offset(2), 'g');
    plot(est_head(1,:) + offset(1), est_head(2,:) + offset(2), 'g-', 'LineWidth', 2);

    % Goal marker
    plot(goal(2), goal(1), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

    % Draw uncertainty ellipses
    draw_ellipse(est_pose(1:2), robot.slam.Sigma(1:2,1:2), 'b');
    for j = 1:length(robot.slam.seen_landmarks)
        if robot.slam.seen_landmarks(j)
            idx = 3 + 2*(j-1) + 1;
            mu_j = robot.slam.mu(idx:idx+1);
            cov_j = robot.slam.Sigma(idx:idx+1, idx:idx+1);
            draw_ellipse(mu_j, cov_j, 'c');
        end
    end
    legend('Landmarks', 'True Pose', 'EKF Estimate', 'Goal');

    % Subplot 2: Thresholded Probability Map
    displayProbabilityMap(prob_map, [1, 2, 2]);
end
