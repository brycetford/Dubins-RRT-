clc; clear; close all;

iter = 1000;

goal_radius = 3;
step_size = 3;

domain = [0 100; 0, 100];

%Obstacles
obstacles = struct();
obstacles.number = 3;
obstacles.type = 1; % AABB - 0, Closed Polygon - 1
obstacles.vertices{1} = [20,30; 40,50; 30,90; 25,85];
obstacles.vertices{2} = [50,25; 80,25; 80,45; 50,40]; 
obstacles.vertices{3} = [11,2; 11,20; 37 30; 39 2];

s = [5.64, 42.96];
t = [80.06, 55.24];
t_idx = -1;

%Plot search space
figure(1);
hold on;
drawdomain(domain,  'k', 2);
plot_obstacles(obstacles, figure(1));

%Plot start and goal
plot(s(1), s(2), 'o', 'MarkerSize', 6, 'Color', 'r');
plot(t(1), t(2), 'o', 'MarkerSize', 6, 'Color', 'g');

tree = struct();
tree.node = zeros(iter, 2);
tree.parent = zeros(iter, 1);
tree.cost = zeros(iter, 1);

tree.node(1, :) = s;
tree.parent(1) = -1;
tree.cost(1) = 0;

i = 2;
reached_goal = false;
while i < iter
    % Generate new point in the domain
    node = [
        (domain(1, 2) - domain(1, 1))*rand() + domain(1, 1),...
        (domain(2, 2) - domain(2, 1))*rand() + domain(2, 1)
        ];
    
    % Find the index and distance to the closest node in the tree
    [parent_dist, parent_idx] = min(vecnorm(tree.node(1:i-1, :) - node, 2, 2));
    
    % Pick the nearest on the connection that is within the step size
    if parent_dist > step_size
        alpha = atan2(node(2) - tree.node(parent_idx, 2), node(1) - tree.node(parent_idx, 1));
        node = [tree.node(parent_idx, 1) + step_size*cos(alpha), tree.node(parent_idx, 2) + step_size*sin(alpha)];
        parent_dist = step_size;
    end
    
    % If the node is admissible, add it to the tree
    if check_admissibility(obstacles, tree.node(parent_idx, :), node)
        tree.node(i, :) = node;
        tree.parent(i) = parent_idx;
        tree.cost(i) = tree.cost(parent_idx) + parent_dist;
        
        % Plot the new node on the tree
        plot(node(1), node(2), '.', 'Color', 'r', 'MarkerSize', 10);
        plot([tree.node(parent_idx, 1); node(1)], [tree.node(parent_idx, 2); node(2)], 'Color', 'b');
        
        % Check if the goal has been reached
        if norm(node - t) <= goal_radius
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
    p_list = tree.node(p_idx, :);
    while tree.parent(p_idx) ~= -1
        p_list = [p_list; tree.node(tree.parent(p_idx), :)];
        p_idx = tree.parent(p_idx);
    end
    plot(p_list(:, 1), p_list(:, 2), 'Color', 'm', 'LineWidth', 4);
end