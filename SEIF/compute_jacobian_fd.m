function F = compute_jacobian_fd(x)
    % x is the current state [x; y; theta]
    % F will be the 3x3 Jacobian matrix
    n = length(x);
    F = zeros(n, n);
    fx = motion_model(x); % f(x)

    epsilon = 1e-6;
    for i = 1:n
        dx = zeros(n, 1);
        dx(i) = epsilon;

        fx_plus = motion_model(x + dx);
        fx_minus = motion_model(x - dx);

        % Central difference formula (more accurate than forward diff)
        F(:, i) = (fx_plus - fx_minus) / (2 * epsilon);
    end
end