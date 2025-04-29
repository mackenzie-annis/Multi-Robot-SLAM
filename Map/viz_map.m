imshow(map); hold on;
    plot(landmarks(:,1), landmarks(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    for i = 1:size(landmarks,1)
        text(landmarks(i,1)+5, landmarks(i,2), num2str(landmarks(i,3)), 'Color', 'red', 'FontSize', 8);
    end