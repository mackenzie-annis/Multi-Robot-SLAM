classdef Sensor < handle
    properties
        range = 500;               % max range in map units (pixels)
        range_min = 1;             % minimum range
        fov = 2*pi;                % 360 field of view
        angular_res = deg2rad(1);  % angular spacing between beams
        sigma_r = 0;             % range noise (map units)
        sigma_phi = deg2rad(0);    % bearing noise (radians)
        last_measurements =[]; % active ids for plotting and debugging
    end

    methods
        % Constructer
        function obj = Sensor(varargin)
            for i = 1:2:length(varargin)
                if isprop(obj, varargin{i})
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
        end


        % Mock Lidar Scan 
        function [ranges, bearings, ideal_ranges] = lidarScan(obj, map, robot_pose)
            [H, W] = size(map);
        
            % Get true robot pose
            cx = robot_pose(1);
            cy = robot_pose(2);
            theta = robot_pose(3);
        
            % Beam angles relative to robot heading
            beam_angles = -obj.fov/2 : obj.angular_res : obj.fov/2;
            n_beams = length(beam_angles);
        
            % Initialize outputs
            ranges = nan(1, n_beams);
            bearings = nan(1, n_beams);
            ideal_ranges = nan(1, n_beams);  % true distance to obstacle
        
            for i = 1:n_beams
                a = beam_angles(i);  % relative angle
                beam_angle = wrapTo2Pi(theta + a);  % global beam angle
        
                % Step outwards along beam
                for r = obj.range_min : obj.range
                    x = round(cx + r * cos(beam_angle));
                    y = round(cy + r * sin(beam_angle));
        
                    % Out of bounds
                    if x < 1 || x > W || y < 1 || y > H
                        break;
                    end
        
                    % Hit wall
                    if map(y, x) == 0
                        ideal_ranges(i) = r;  % true hit range for ray stepping
                        r_noisy = r + randn() * obj.sigma_r;
                        phi_noisy = wrapToPi(a + randn() * obj.sigma_phi);
                        ranges(i) = max(obj.range_min, min(obj.range, r_noisy));
                        bearings(i) = phi_noisy;
                        break;
                    end
                end
        
                % No hit within range
                if isnan(ranges(i))
                    ranges(i) = obj.range;                % max range
                    ideal_ranges(i) = obj.range;          % mark as full ray
                    bearings(i) = wrapToPi(a);            % still store nominal angle
                end
            end
        end

        
        % Gets lidar hit points in the global coord frame
        function hit_pts = getLidarHitPoints(obj, robot_pose, map)
            [ranges, bearings] = obj.lidarScan(map, robot_pose);
            [xg, yg] = obj.localToGlobalPolar(robot_pose, ranges, bearings);
            hit_pts = [xg(:)'; yg(:)'];  % [2 x N]
        end

        % Since the landmarks are assumed to be identifiable I will scan
        % them in a different way instead of doing feature matching
        function Z = measureLandmarks(obj, robot_pose, landmarks, map)
            % Assumes landmarks = [x y id]
            Z = [];
            for i = 1:size(landmarks, 1)
                lx = landmarks(i,1);
                ly = landmarks(i,2);
                id = landmarks(i,3);

                dx = lx - robot_pose(1);
                dy = ly - robot_pose(2);
                r = sqrt(dx^2 + dy^2);
                phi = wrapToPi(atan2(dy, dx) - robot_pose(3));
                
                % Use the bresenham line to verify scan not obstructed
                if r <= obj.range && obj.isVisible(map, robot_pose(1), robot_pose(2), lx, ly)
                    r_noisy = r + randn()*obj.sigma_r;
                    phi_noisy = wrapToPi(phi + randn()*obj.sigma_phi);
                    Z(end+1, :) = [r_noisy, phi_noisy, id];
                end
            end
        end
        
        % Landmarks in global coord frame 
        function [meas_pts, Z] = getLandmarkMeasurements(obj, robot_pose, landmarks, map)
            % Measure the landmarks and get the raw measurements
            Z = obj.measureLandmarks(robot_pose, landmarks, map);  % [r, phi, id]
            
            % If there are no measurements, return empty
            if isempty(Z)
                meas_pts = [];
                return;
            end
            
            % Convert measurements to global coordinates
            [xg, yg] = obj.localToGlobalPolar(robot_pose, Z(:,1), Z(:,2));
            meas_pts = [xg(:)'; yg(:)'];  % [2 x N]
        end



        function visible = isVisible(~, map, x1, y1, x2, y2)
            pts = Sensor.bresenham(round(x1), round(y1), round(x2), round(y2));
            visible = true;
            for k = 1:size(pts, 1)
                x = pts(k,1); y = pts(k,2);
                if x < 1 || y < 1 || y > size(map,1) || x > size(map,2) || map(y,x) == 0
                    visible = false;
                    return;
                end
            end
        end
    end

    methods (Static)
        function pts = bresenham(x0, y0, x1, y1)
            dx = abs(x1 - x0); dy = abs(y1 - y0);
            sx = sign(x1 - x0); sy = sign(y1 - y0);
            err = dx - dy; pts = [];
            while true
                pts(end+1,:) = [x0, y0];
                if x0 == x1 && y0 == y1, break; end
                e2 = 2 * err;
                if e2 > -dy, err = err - dy; x0 = x0 + sx; end
                if e2 < dx, err = err + dx; y0 = y0 + sy; end
            end
        end

        function [xg, yg] = localToGlobalPolar(robot_pose, ranges, bearings)
            % Converts local polar [range, bearing] to global [x, y]
            theta = robot_pose(3);
            x = robot_pose(1);
            y = robot_pose(2);
        
            global_bearings = bearings + theta;
            [dx, dy] = pol2cart(global_bearings, ranges);
        
            xg = x + dx;
            yg = y + dy;
        end

    end
end
