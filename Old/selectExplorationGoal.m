function goal = selectExplorationGoal(log_odds_map)
    prob_map = 1 - 1 ./ (1 + exp(log_odds_map));
    
    entropy_map = -prob_map .* log(prob_map + eps) - (1 - prob_map) .* log(1 - prob_map + eps);
    [~, idx] = max(entropy_map(:));
    [y, x] = ind2sub(size(entropy_map), idx);
    goal = [x; y];
end
