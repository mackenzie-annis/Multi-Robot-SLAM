function inflated = inflateProbMap(prob_map, inflation_radius)
% inflateProbMap - Inflates only occupied regions in a probability map
% 
% Inputs:
%   prob_map         - HxW matrix with values in [0, 1]
%   inflation_radius - radius in pixels for obstacle inflation
%
% Output:
%   inflated - HxW matrix of same size
%             occupied = 1.0
%             free     = 0.0
%             unknown  = 0.5 

    % Step 1: Identify occupied pixels
    occ_mask = prob_map > 0.5;

    % Step 2: Dilate occupied regions
    se = strel('disk', inflation_radius);
    inflated_occ = imdilate(occ_mask, se);

    % Step 3: Construct final map
    inflated = prob_map;
    inflated(prob_map < 0.5) = 0.0;  % mark free (black)
    inflated(prob_map == 0.5) = 0.5;  
    inflated(inflated_occ) = 1.0;  % mark as occupied (white)
end
