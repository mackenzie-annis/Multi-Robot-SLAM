function [fused_mu, fused_Sigma, seen_landmark] = fuse_landmark_estimates( ...
    mu1, Sigma1, mu2, Sigma2, seen1, seen2, landmark_idx)

    fused_mu = [];
    fused_Sigma = [];
    seen_landmark = seen1;  % default

    idx = 3 + 2 * (landmark_idx - 1) + 1;

    has1 = seen1(landmark_idx);
    has2 = seen2(landmark_idx);

    % === Case 3: Neither has seen it
    if ~has1 && ~has2
        return;
    end

    % === Case 2: Only one has seen it → copy it
    if has1 && ~has2
        fused_mu = mu1(idx:idx+1);
        fused_Sigma = Sigma1(idx:idx+1, idx:idx+1);
        seen_landmark = true;
        return;
    elseif has2 && ~has1
        fused_mu = mu2(idx:idx+1);
        fused_Sigma = Sigma2(idx:idx+1, idx:idx+1);
        seen_landmark = true;
        return;
    end

    % === Case 1: Both have seen it → fuse
    l1 = mu1(idx:idx+1);
    l2 = mu2(idx:idx+1);
    S1 = Sigma1(idx:idx+1, idx:idx+1);
    S2 = Sigma2(idx:idx+1, idx:idx+1);

    if norm(l1 - l2) > 100
        return;
    end

    if trace(S1) > 1e6 || trace(S2) > 1e6 || trace(S1) < 1e-6 || trace(S2) < 1e-6
        return;
    end

    w1 = 1 / max(trace(S1), 1e-6);
    w2 = 1 / max(trace(S2), 1e-6);
    w1 = w1 / (w1 + w2);
    w2 = 1 - w1;

    fused_mu = w1 * l1 + w2 * l2;
    fused_Sigma = inv(w1 * inv(S1) + w2 * inv(S2));
    seen_landmark = true;
end


