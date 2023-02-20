clc; clear; close all;

iter = 1000;

goal_radius = 5;
step_size = 8;
R = 3;

domain = [0 100; 0, 100];

%Obstacles
obstacles = struct();
obstacles.number = 3;
obstacles.type = 1; % AABB - 0, Closed Polygon - 1
obstacles.vertices{1} = [20,30; 40,50; 30,90; 25,85];
obstacles.vertices{2} = [50,25; 80,25; 80,45; 50,40]; 
obstacles.vertices{3} = [11,2; 11,20; 37 30; 39 2];

s = [5.64, 42.96, 5.3];
t = [80.06, 55.24, 1.35];
t_idx = -1;

% Plot search space
figure(1);
hold on;
drawdomain(domain,  'k', 2);
plot_obstacles(obstacles, figure(1));

% Plot start and goal
plot(s(1), s(2), 'go', 'Markersize', 10, 'Markerfacecolor', 'g');
quiver(s(1), s(2), cos(s(3))*6, sin(s(3))*6, 'g', 'linewidth', 3);

plot(t(1), t(2), 'ro', 'Markersize', 10, 'Markerfacecolor', 'r');
quiver(t(1), t(2), cos(t(3))*6, sin(t(3))*6, 'r', 'linewidth', 3);

tree = struct();
tree.node = zeros(iter, 3); % (x, y, yaw)
tree.parent = zeros(iter, 1);
tree.cost = zeros(iter, 1);
tree.path = struct('type', [], 'beta_0', [], 'beta_1', [], 's_dist', [], 'cost', []); % defined using: lsl, rsr, rsl, lsr path primitives

% Null dubins path to represent start
start_path = struct();
start_path.type = 0;
start_path.beta_0 = 0;
start_path.beta_1 = 0;
start_path.s_dist = 0;
start_path.cost = 0;

tree.node(1, :) = s;
tree.parent(1) = -1;
tree.cost(1) = 0;
tree.path(1) = start_path;

i = 2;
reached_goal = false;
while i < iter
    % Generate new point in the domain
    node = [
            (domain(1, 2) - domain(1, 1))*rand() + domain(1, 1),...
            (domain(2, 2) - domain(2, 1))*rand() + domain(2, 1),...
            2*pi*rand()
        ];
    
    % Find the index and distance to the closest node in the tree
    [parent_dist, parent_idx] = min(vecnorm(tree.node(1:i-1, 1:2) - node(1:2), 2, 2));
    
    % Pick the nearest on the connection that is within the step size
    if parent_dist > step_size
        alpha = atan2(node(2) - tree.node(parent_idx, 2), node(1) - tree.node(parent_idx, 1));
        node(1:2) = [tree.node(parent_idx, 1) + step_size*cos(alpha), tree.node(parent_idx, 2) + step_size*sin(alpha)];
        parent_dist = step_size;
    end
    
    % Get dubins path between tree node and new node
    path = generate_dubins(tree.node(parent_idx, :), node, R);
    
    % If the node is admissible, add it to the tree
    if check_admissibility_dubins(tree.node(parent_idx, :), node, path, R, obstacles)
        tree.node(i, :) = node;
        tree.parent(i) = parent_idx;
        tree.cost(i) = tree.cost(parent_idx) + path.cost;
        tree.path(i) = path;
        
        % Plot the new node on the tree
        points = generate_points(tree.node(parent_idx, :), node, path, R);
        hold on;
        plot(node(1), node(2), '.', 'Color', 'r', 'MarkerSize', 4);
        quiver(node(1), node(2), cos(node(3))*2, sin(node(3))*2, 'r', 'linewidth', 1);
        plot(points(:, 1), points(:, 2), 'Color', 'b');
        
        % Check if the goal has been reached
        if norm(node(1:2) - t(1:2)) <= goal_radius
            t_idx = i;
            reached_goal = true;
            fprintf("Goal Reached!\n");
            break;
        end
        
        i = i + 1;
    end
end

if reached_goal
    p_idx = t_idx;
    p_list = [];
    while tree.parent(p_idx) ~= -1
        points = generate_points(tree.node(tree.parent(p_idx), :), tree.node(p_idx, :), tree.path(p_idx), R);
        p_list = [points; p_list];
        p_idx = tree.parent(p_idx);
    end
    plot(p_list(:, 1), p_list(:, 2), 'Color', 'm', 'LineWidth', 4);
end
