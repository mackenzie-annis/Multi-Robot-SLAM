function x_new = apply_noisy_motion(x, u, dt, alpha)
    v = u(1); w = u(2);
    theta = x(3);

    % Add noise
    v_hat = v + randn()*alpha(1);
    w_hat = w + randn()*alpha(2);

    w_hat = max([w_hat, 1e-4]); % avoid division by 0 

    x_new = [
        x(1) - (v_hat/w_hat)*sin(theta) + (v_hat/w_hat)*sin(theta + w_hat*dt);
        x(2) + (v_hat/w_hat)*cos(theta) - (v_hat/w_hat)*cos(theta + w_hat*dt);
        wrapToPi(theta + w_hat*dt)
    ];
end