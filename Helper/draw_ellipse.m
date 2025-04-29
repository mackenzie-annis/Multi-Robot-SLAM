function draw_ellipse(mu, Sigma, color)
    if nargin < 3
        color = 'r';
    end

    theta = linspace(0, 2*pi, 100);
    unit_circle = [cos(theta); sin(theta)];

    [evec, evals] = eig(Sigma);
    if any(diag(evals) < 0)
        return  % skip invalid (non-PD) matrices
    end

    ellipse = evec * sqrt(evals) * unit_circle;
    plot(mu(1) + ellipse(1,:), mu(2) + ellipse(2,:), color, 'LineWidth', 1.5);
end
