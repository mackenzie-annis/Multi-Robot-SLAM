function config = configuration()
    % Initial onfiguration for multi-robot exploration
    % Load Map
    map_data = load('Map/binary_map.mat');
    config.map = map_data.map;
    landmarks = map_data.landmarks;
    config.landmarks = [landmarks, (1:size(landmarks,1))']; 
    config.N = length(config.landmarks); % number of landmarks

    %% ROBOT Settings (Including Measurement Model and SlAM) 
    % Robot Initial Settings
    num_robots = 2;
    config.num_robots = num_robots;
    config.robots = cell(1, config.num_robots); % Cell array to hold robots

    % All start at same location not accounting for collisions
    initial_poses = repmat([100; 100; pi/2], 1, num_robots); % all start at same location
    
    % SLAM Settings 
 
    
    % Set up each robot
    for i = 1:num_robots
        % Instantiate robot just put initial pose, sensor, and slam
        % instance, the reset keep as default
        % Create sensor instance (Can customize per robot)
        sensor = Sensor('range', 100, ...
                                   'sigma_r', 1.0, ...
                                   'sigma_phi', deg2rad(1), ...
                                   'angular_res', deg2rad(0.1));
        % Create SLAM instance 
        slam = SLAM(initial_poses(:,i), size(config.landmarks,1));
        config.robots{i} = Robot('pose', initial_poses(:,i), 'sensor', sensor, 'slam', slam, 'map_size', size(config.map));
    end

    %% Simulation Parameters
    % A* Settings
    config.astar = struct('inflation_radius', 10, 'penalty_inside', 10, 'penalty_entering', 1);
    config.max_steps = 1000;
end
