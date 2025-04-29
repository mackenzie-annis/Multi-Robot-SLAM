clear; clc; close all;
addpath(genpath('~/Documents/MATLAB/autonomous_robots'));

% === CONFIGURATION ===
config = configuration();
robot = config.robots{1};
map = config.map;
landmarks = config.landmarks;
log_odds_map = robot.log_odds_map;

last_goal = [];

% === MAIN LOOP ===
for t = 1:5000
    % --- SLAM & MAPPING ---
    robot.updateMapAndSLAM(landmarks, map);
    log_odds_map = robot.log_odds_map;
    prob_map = 1 - 1 ./ (1 + exp(log_odds_map));
    prob_map = inflateProbMap(prob_map,3);
 
    % --- FRONTIER GOAL SELECTION ---
    robot_pos = round(flip(robot.pose(1:2)));
    goal = select_goal(prob_map, last_goal, robot_pos, 10);
    last_goal = goal;


    % --- PLAN PATH ---
    inflated_map = inflateProbMap(prob_map, config.astar.inflation_radius);
    path = astar(inflated_map, prob_map > 0.5, robot_pos, goal);

    % --- MOVE ONE STEP ---
    if size(path, 1) >= 2
        next = path(2, :);
        dx = next(2) - robot_pos(2);
        dy = next(1) - robot_pos(1);
        angle_to_next = atan2(dy, dx);
        theta = robot.pose(3);
        robot.setControl([5; angle_to_next - theta]);
        robot.applyNoisyMotion(map);
    end
    
    % Plotting 
    plotExplorationStep(robot, goal, map, landmarks, prob_map, t)
end
