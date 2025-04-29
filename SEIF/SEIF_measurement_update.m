function [xi, omega, seen_landmarks] = SEIF_measurement_update(xi, omega, mu, zt, ct, sigma_r, sigma_phi, sigma_s, seen_landmarks)
% Inputs:
% xi, omega: information vector/matrix
% mu: state vector mean [x; y; theta; landmarks(1-N)]
% zt: measuremements
% ct: correspondance matrix
% sigma_r, sigma_phi, sigma_s: measurement noise
% seen_landmarks: array tracking initialized landmarks

    Qt = diag([sigma_r^2, sigma_phi^2, sigma_s^2]);
    
    % Loop through all observed features
    for i = 1:size(zt, 1)
        % extract landmark 
        z_i = zt(i,:)';
        r_i = z_i(1);
        phi_i = z_i(2);
        s_i = z_i(3);

        j = ct(i); % landmark index (known correspondence so it has an ID for simplified example) 
        idx = 3 + 3*j - 1; % index in comb state matrices

        if ~seen_landmarks(j)
            % Initialize landmark in state
            mu(idx)   = mu(1) + r_i * cos(phi_i + mu(3));
            mu(idx+1) = mu(2) + r_i * sin(phi_i + mu(3));
            mu(idx+2) = s_i; 
            seen_landmarks(j) = true;
        end

        % Innovation - diff between robot and landmark
        delta = [mu(idx) - mu(1); % delta x
                 mu(idx+1) - mu(2)]; % delta y 

        q = delta(1)^2 + delta(2)^2;  % Squared dist from ^

        % Expected measurement
        z_hat = [sqrt(q);
                 atan2(delta(2), delta(1)) - mu(3);
                 mu(idx+2)];

        % Jacobian H_i (sparse form) only robot pose and landmark j
        % influence measurement (linearizes for update)
        sqrtq = sqrt(q);
        H_i = zeros(3, length(mu));
        H_i(1,1) = delta(1)/sqrtq;
        H_i(1,2) = -delta(2)/sqrtq;
        H_i(1,idx) = -delta(1)/sqrtq;
        H_i(1,idx+1) = delta(2)/sqrtq;

        H_i(2,1) = delta(2)/q;
        H_i(2,2) = delta(1)/q;
        H_i(2,3) = -1;
        H_i(2,idx) = -delta(2)/q;
        H_i(2,idx+1) = -delta(1)/q;
        H_i(3,idx+2) = 1;  % s_j index

        % Info update incorporating measurement information into belief
        % about the state (adds sparse structure)
        inv_Q = inv(Qt);
        
        xi = xi +  H_i' * inv_Q * (z_i - z_hat - H_i * mu);
        omega = omega + H_i' * inv_Q * H_i;
    end
end
