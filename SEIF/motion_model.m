function x_next = motion_model(x)
    % x = [x; y; theta]
    % Your assignment's motion model with omega = 0.1 + x^2
    % v = 1, dt = 1

    v = 1;
    dt = 1;
    x_pos = x(1);
    y_pos = x(2);
    theta = x(3);
    omega = 0.1 + x_pos^2;

    x_next = zeros(3,1);
    x_next(1) = x_pos + (v/omega) * (sin(theta + omega*dt) - sin(theta));
    x_next(2) = y_pos + (v/omega) * (-cos(theta + omega*dt) + cos(theta));
    x_next(3) = theta + omega * dt;
end
