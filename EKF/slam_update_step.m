function [mu, Sigma, seen_landmarks] = slam_update_step(robot, mu, Sigma, u,  map, landmarks, seen_landmarks)
    % 1. Motion update
    motion_noise = robot.noise(); %2x1 noise vector for v and omega 
    R_t = diag([motion_noise(1)^2, motion_noise(1)^2, motion_noise(1)^2]);
    N = length(landmarks);
    [mu, Sigma] = ekf_motion_update(mu, Sigma, u, R_t, robot.dt, N);

    % 2. Get landmark measurements (assumes known data association)
    sensor = robot.sensor;
    % Landmarks in global coord frame 
    [meas_pts, ~] = sensor.getLandmarkMeasurements(robot.pose, landmarks, map);
    Z = sensor.measureLandmarks(robot.pose, landmarks, map);
    % Update last_measurements with the global coordinates
    sensor.last_measurements = meas_pts;

    % 3. Measurement update
    [mu, Sigma, seen_landmarks] = ekf_slam_update(mu, Sigma, Z, ...
        sensor.sigma_r, sensor.sigma_phi, seen_landmarks);
end
