classdef SLAM < handle
    properties
        mu                % State vector
        Sigma             % Covariance matrix
        seen_landmarks    % Bool array tracking initialized landmarks
        num_landmarks 
    end

    methods
        % CONSTRUCTOR
        function obj = SLAM(initial_pose, num_landmarks)
            % Initialize mu to initial posa and zeroes for the landmarks
            obj.mu = zeros(3 + 2*num_landmarks, 1);
            obj.mu(1:3) = initial_pose;
    
            obj.Sigma = eye(length(obj.mu)) * 1e6; % Large uncertainty of all landmarys 
            obj.Sigma(1:3,1:3) = 1e-3 * eye(3); % Small uncertainty for the inital robot pose 
    
            obj.seen_landmarks = false(1, num_landmarks); %lanadmark tracker
        end
         
        % SLAM Update Step 
        function update(obj,robot, u, landmarks, map)
            [mu, Sigma, seen] = slam_update_step(robot, obj.mu, obj.Sigma, u, map, landmarks, obj.seen_landmarks);
            
            obj.mu = mu;
            obj.Sigma = Sigma;
            obj.seen_landmarks = seen;
        end

        function pose = getPose(obj)
            pose = obj.mu(1:3);
        end
    end
end

