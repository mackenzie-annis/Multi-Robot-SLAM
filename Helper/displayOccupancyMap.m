function displayOccupancyMap(log_odds_map, subplot_index)
% Displays a thresholded occupancy grid map based on log-odds values.
% Inputs:
%   log_odds_map   - the current log-odds occupancy grid (HxW)
%   subplot_index  - which subplot to use (e.g., [1,2,2] for subplot(1,2,2))

    % Convert log-odds to probability
    prob_map = 1 - 1 ./ (1 + exp(log_odds_map));

    % Initialize display map with gray (unexplored)
    display_map = 0.5 * ones(size(prob_map));

    % Assign values
    display_map(prob_map < 0.5) = 0;  % free -> black (0)
    display_map(prob_map > 0.5) = 1;  % occupied -> white (1)

    % Show in subplot
    subplot(subplot_index(1), subplot_index(2), subplot_index(3));
    imagesc(display_map)
    colormap(gca, gray);              % black = 0, white = 1
    axis image;
    title('Occupancy Grid');
    set(gca, 'XTick', [], 'YTick', []);
end