classdef Robot < handle
    properties
        pose        % True pose [x; y; theta]
        u           % Control input [v; w]
        noise       % Motion noise [sigma_v; sigma_w]
        dt          % Timestep
        rad    % radius of robot (robot displayed a cirve with heading dir
        sensor % Sensor instance
        slam % SLAM instance
        role % explorer, helper, pause
        seen_landmarks % list of landmarks this robot has seen
        log_odds_map % local log odds map
        goal % current navigation goal
        frontiers
        centroids
        path % A* generatied
        
    end

    methods
        % Constructer 
        function rob = Robot(varargin)
            % Default values
            default_pose = [100; 100; pi/4];
            default_dt = 1.0;
            default_noise = [0.1; 0.01]; % motion noise
            default_sensor = [];
            default_slam = [];
            default_role = 'explorer';
            default_rad = 15;
            default_map_size = [800, 1000]; % used to init local log_odds_map

            % Parse input parameters
            p = inputParser;
            addParameter(p, 'pose', default_pose);
            addParameter(p, 'dt', default_dt);
            addParameter(p, 'noise', default_noise);
            addParameter(p, 'sensor', default_sensor);
            addParameter(p, 'slam', default_slam);
            addParameter(p, 'role', default_role);
            addParameter(p, 'rad', default_rad);
            addParameter(p, 'map_size', default_map_size);
            parse(p, varargin{:});

            % Assign properties
            rob.pose = p.Results.pose;
            rob.dt = p.Results.dt;
            rob.noise = p.Results.noise;
            rob.sensor = p.Results.sensor;
            rob.slam = p.Results.slam;
            rob.role = p.Results.role;
            rob.rad = p.Results.rad;
            map_size = p.Results.map_size;
            rob.log_odds_map = zeros(map_size);

            rob.u = [0; 0]; % Initial control input
            rob.goal = [0; 0];
            rob.seen_landmarks = [];
            rob.frontiers = [];
            rob.centroids = [];
            rob.path = [];
        end
        
        % Mutator control
        function obj = setControl(obj, u)
            obj.u = u;
        end
    
        % Apply noisy motion to get true pose
        function pose = applyNoisyMotion(obj, map)
            v = obj.u(1);
            w = obj.u(2);
            theta = obj.pose(3);
        
            v_noisy = v + obj.noise(1)*randn();
            w_noisy = w + obj.noise(2)*randn();
        
            epsilon = 1e-6;
            if abs(w_noisy) < epsilon
                dx = v_noisy * cos(theta) * obj.dt;
                dy = v_noisy * sin(theta) * obj.dt;
                dtheta = 0;
            else
                dx = -(v_noisy/w_noisy)*sin(theta) + (v_noisy/w_noisy)*sin(theta + w_noisy*obj.dt);
                dy =  (v_noisy/w_noisy)*cos(theta) - (v_noisy/w_noisy)*cos(theta + w_noisy*obj.dt);
                dtheta = w_noisy * obj.dt;
            end
        
            new_x = obj.pose(1) + dx;
            new_y = obj.pose(2) + dy;
        
            % Check if path is free using Bresenham line tracing
            [x0, y0] = deal(round(obj.pose(1)), round(obj.pose(2)));
            [x1, y1] = deal(round(new_x), round(new_y));
            pts = Sensor.bresenham(x0, y0, x1, y1);
            
            is_free = true;
            for i = 1:size(pts,1)
                x = pts(i,1); y = pts(i,2);
                if x < 1 || y < 1 || y > size(map,1) || x > size(map,2) || map(y,x) == 0
                    is_free = false;
                    disp("debug")
                    break;
                end
            end

        if is_free
            obj.pose = [new_x; new_y; obj.pose(3) + dtheta];
        end
        
            pose = obj.pose;
        end

        function updateMapAndSLAM(obj, landmarks, map)
            % Run SLAM update
            obj.slam.update(obj, obj.u, landmarks, map);
            % Update occupancy map
            obj.log_odds_map = occupancyMapping(obj.log_odds_map, obj.pose, obj.sensor, map);
        end



        function [circle_pts, heading_line] = computePoseShape(obj)
        % Returns robot body (circle) and heading line for plotting later
        % Outputs:
        %   circle_pts: [2 x N] points on the circle
        %   heading_line: [2 x 2] [start; end] of heading vector

            x = obj.pose(1);
            y = obj.pose(2);
            theta = obj.pose(3);
        
            % Circle points
            ang = linspace(0, 2*pi, 100);
            xp = obj.rad * cos(ang);
            yp = obj.rad * sin(ang);
            circle_pts = [x + xp; y + yp];
        
            % Heading line
            heading_len = 1.5 * obj.rad;
            x2 = x + heading_len * cos(theta);
            y2 = y + heading_len * sin(theta);
            heading_line = [x, x2; y, y2];
        end

        function is_free = isFree(map, x, y)
            [H, W] = size(map);
            x = round(x);
            y = round(y);
            if x < 1 || x > W || y < 1 || y > H
                is_free = false;
            else
                is_free = map(y, x) == 1;  % Assuming 1 = free space, 0 = obstacle
            end
        end

    end
    
end