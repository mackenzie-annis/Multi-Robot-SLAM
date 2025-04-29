function [xi_new, omega_new] = SEIF_sparsification(xi, omega, omega_0t, mu, F_xmo, F_mo, F_x)
    % Inputs:
    % omega: information matrix
    % omega_0t: prior sparse approximation
    % mu : current mean 
    % F_: selection matrices
    
    % Sparsified information matrix
    omega_new = omega - omega_0t * F_mo * inv(F_mo' * omega_0t * F_mo) * F_mo'* omega_0t ...
        + omega_0t * F_xmo * inv(F_xmo' * omega_0t * F_xmo) *  F_xmo' * omega_0t ...
        - omega * F_x * inv(F_x' * omega * F_x) * F_x' * omega;

    % Adjusted information vector
    xi_new = xi + mu(omega_new - omega);
end


% Sparsification - Removes weak dependencies between variables ie) dist
% landmarks 
% project info matrix + vector into sparser form using marginalization 

% F_x selects state, F_mo selects robot + landmarks retained, F_xmo
% selectes var to keep in "some" way