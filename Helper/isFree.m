function is_free = isFree(map, x, y)
    [H, W] = size(map);
    x = round(x);
    y = round(y);
    if x < 1 || x > W || y < 1 || y > H
        is_free = false;
    else
        is_free = map(y, x) == 1;  % Assuming 1 = free space, 0 = obstacle
    end
end
