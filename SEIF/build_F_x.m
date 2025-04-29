function F_x = build_F_x(mu)
    % Selects robot pose (first 3 variables)
    F_x = sparse(3, length(mu));
    F_x(1,1) = 1;
    F_x(2,2) = 1;
    F_x(3,3) = 1;
end