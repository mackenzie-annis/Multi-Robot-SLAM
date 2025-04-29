clear; clc;
filename = 'map.png'; 
resize_to = [600, 800];     % Final map size in pixels (height x width)
threshold = 0.5;            % Threshold for binarization (0-1 grayscale)

%load img
img = imread(filename);
if size(img, 3) == 3
    img = rgb2gray(img);
end

% Resize
img = imresize(img, resize_to);

% Convert to binary (0 = black = wall, 1 = white = free)
map = imbinarize(im2double(img), threshold);

% Display 
figure;
imshow(map, 'InitialMagnification', 'fit');
colormap(gray);
title('Click to add landmarks. Press Enter when done.');
hold on;

% Get landmark clicks
[x, y] = ginput(); % Clicks, press Enter when done
landmarks = [x, y];

% Plot landmarks
plot(x, y, 'ro', 'MarkerSize', 8, 'LineWidth', 2);

% Save for SLAM
save('binary_map.mat', 'map', 'landmarks');


