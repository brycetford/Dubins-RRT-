function [path] = generate_dubins(start, goal, R)
% Returns minimum cost dubins path connecting 2 points if possible given
% turn radius R
% Using: rsr, lsl, rsl, rsl path primitives

path_list = zeros(4, 4);

% Generate circle centers
center_0_l = [start(1) - R*sin(start(3)), start(2) + R*cos(start(3))];
center_0_r = [start(1) + R*sin(start(3)), start(2) - R*cos(start(3))];
center_1_l = [goal(1) - R*sin(goal(3)), goal(2) + R*cos(goal(3))];
center_1_r = [goal(1) + R*sin(goal(3)), goal(2) - R*cos(goal(3))];

% lsl
s_dist = norm(center_0_l - center_1_l);
alpha = atan2(center_1_l(2) - center_0_l(2), center_1_l(1) - center_0_l(1));
beta_1 = mod(goal(3) - alpha, 2*pi);
beta_0 = mod(alpha - start(3), 2*pi);
cost = R*(beta_0 + beta_1) + s_dist;
path_list(1, :) = [s_dist, beta_0, beta_1, cost];

% rsr
s_dist = norm(center_0_r - center_1_r);
alpha = atan2(center_1_r(2) - center_0_r(2), center_1_r(1) - center_0_r(1));
beta_1 = mod(-goal(3) + alpha, 2*pi);
beta_0 = mod(-alpha + start(3), 2*pi);
cost = R*(beta_0 + beta_1) + s_dist;
path_list(2, :) = [s_dist, -beta_0, -beta_1, cost];

% rsl
med_point = (center_1_l - center_0_r)./2;
half_intercenter = norm(med_point);
if half_intercenter < R
    path_list(3, :) = [0, 0, 0, inf];
else
    psi = atan2(med_point(2), med_point(1));
    alpha = acos(R/half_intercenter);
    beta_0 = mod(-(psi + alpha - start(3) - pi/2), 2*pi);
    beta_1 = mod(pi/2 + goal(3) - alpha - psi, 2*pi);
    s_dist = 2*(half_intercenter^2 - R^2)^0.5;
    cost = R*(beta_0 + beta_1) + s_dist;
    path_list(3, :) = [s_dist, -beta_0, beta_1, cost];
end

% lsr
med_point = (center_1_r - center_0_l)./2;
half_intercenter = norm(med_point);
if half_intercenter < R
    path_list(4, :) = [0, 0, 0, inf];
else
    psi = atan2(med_point(2), med_point(1));
    alpha = acos(R/half_intercenter);
    beta_0 = mod(psi - alpha - start(3) + pi/2, 2*pi);
    beta_1 = mod(pi/2 - goal(3) - alpha + psi, 2*pi);
    s_dist = 2*(half_intercenter^2 - R^2)^0.5;
    cost = R*(beta_0 + beta_1) + s_dist;
    path_list(4, :) = [s_dist, beta_0, -beta_1, cost];
end

[~, min_idx] = min(path_list(:, 4));

path = struct();
path.type = min_idx;
path.s_dist = path_list(min_idx, 1);
path.cost = path_list(min_idx, 4);
path.beta_0 = path_list(min_idx, 2);
path.beta_1 = path_list(min_idx, 3);
end

