function manageRobotBehaviors(robots, uncertainty_threshold)
% Updates each robot's role: 'explorer', 'pause', or 'helper'
% - robots: cell array of Robot instances
% - uncertainty_threshold: threshold on trace(Sigma) to pause a robot

num_robots = length(robots);
uncertainties = zeros(1, num_robots);
poses = zeros(2, num_robots);

% Step 1: Check uncertainty
for i = 1:num_robots
    Sigma = robots{i}.slam.Sigma(1:2, 1:2); % use 2D position uncertainty
    uncertainties(i) = trace(Sigma);
    poses(:, i) = robots{i}.slam.mu(1:2); % estimated position
end

% Step 2: Identify robots that need help
paused = false(1, num_robots);
for i = 1:num_robots
    if uncertainties(i) > uncertainty_threshold
        robots{i}.role = 'pause';
        paused(i) = true;
    else
        robots{i}.role = 'explorer';
    end
end

% Step 3: Assign helpers to paused robots
for i = 1:num_robots
    if paused(i)
        % Find the closest available explorer
        dists = vecnorm(poses - poses(:, i), 2, 1);
        dists(i) = inf; % exclude self
        for j = 1:num_robots
            if strcmp(robots{j}.role, 'explorer') && dists(j) < inf
                robots{j}.role = 'helper';
                robots{j}.setGoal(round(flip(poses(:, i)))'); % helper moves to paused robot
                break;
            end
        end
    end
end
end
