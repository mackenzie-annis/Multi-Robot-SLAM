function displayProbabilityMap(prob_map, subplot_index)
% Displays a thresholded occupancy grid map using a probability map.
% Inputs:
%   prob_map       - [HxW] matrix of occupancy probabilities [0,1]
%   subplot_index  - subplot index (e.g., [1,2,1])

    % Initialize display map with gray (unexplored = 0.5)
    display_map = 0.5 * ones(size(prob_map));

    % Thresholds (same logic as displayOccupancyMap)
    display_map(prob_map < 0.5) = 0;   % free -> black
    display_map(prob_map > 0.5) = 1;   % occupied -> white

    % Display
    subplot(subplot_index(1), subplot_index(2), subplot_index(3));
    imagesc(display_map);
    colormap(gca, gray);
    axis image;
    title('Thresholded Probability Map');
    set(gca, 'XTick', [], 'YTick', []);
end