function [points] = generate_points(start, goal, path, R)
% Given start and goal and connecting dubins path, discritize and return
% points along the path
N_0 = max(3, ceil(R*abs(path.beta_0)));
N_1 = max(3, ceil(path.s_dist/4));
N_2 = max(3, ceil(R*abs(path.beta_1)));

N = N_0 + N_1 + N_2;
points = zeros(N, 2);

switch path.type
    case 1 % lsl
        center_0 = [start(1) - R*sin(start(3)), start(2) + R*cos(start(3))];
        center_1 = [goal(1) - R*sin(goal(3)), goal(2) + R*cos(goal(3))];
        points(1:N_0, :) = [ 
                center_0(1) + R*sin(linspace(start(3), start(3) + path.beta_0, N_0)'),...
                center_0(2) - R*cos(linspace(start(3), start(3) + path.beta_0, N_0)')
            ];
        points(N_0+N_1+1:N, :) = [
                center_1(1) + R*sin(linspace(goal(3) - path.beta_1, goal(3), N_2)'),...
                center_1(2) - R*cos(linspace(goal(3) - path.beta_1, goal(3), N_2)')
            ];
        
        s_x = linspace(points(N_0, 1), points(N_0+N_1+1, 1), N_1 + 2);
        s_y = linspace(points(N_0, 2), points(N_0+N_1+1, 2), N_1 + 2);
        points(N_0+1:N_0+N_1, :) = [s_x(2:end-1)', s_y(2:end-1)'];
        
    case 2 % rsr
        center_0 = [start(1) + R*sin(start(3)), start(2) - R*cos(start(3))];
        center_1 = [goal(1) + R*sin(goal(3)), goal(2) - R*cos(goal(3))];
        points(1:N_0, :) = [ 
                center_0(1) + R*sin(-linspace(start(3), start(3) + path.beta_0, N_0)'),...
                center_0(2) + R*cos(-linspace(start(3), start(3) + path.beta_0, N_0)')
            ];
        points(N_0+N_1+1:N, :) = [
                center_1(1) + R*sin(-linspace(goal(3) - path.beta_1, goal(3), N_2)'),...
                center_1(2) + R*cos(-linspace(goal(3) - path.beta_1, goal(3), N_2)')
            ];
        
        s_x = linspace(points(N_0, 1), points(N_0+N_1+1, 1), N_1 + 2);
        s_y = linspace(points(N_0, 2), points(N_0+N_1+1, 2), N_1 + 2);
        points(N_0+1:N_0+N_1, :) = [s_x(2:end-1)', s_y(2:end-1)'];
        
    case 3 % rsl
        center_0 = [start(1) + R*sin(start(3)), start(2) - R*cos(start(3))];
        center_1 = [goal(1) - R*sin(goal(3)), goal(2) + R*cos(goal(3))];
        points(1:N_0, :) = [ 
                center_0(1) + R*sin(-linspace(start(3), start(3) + path.beta_0, N_0)'),...
                center_0(2) + R*cos(-linspace(start(3), start(3) + path.beta_0, N_0)')
            ];
        points(N_0+N_1+1:N, :) = [
                center_1(1) + R*sin(linspace(goal(3) - path.beta_1, goal(3), N_2)'),...
                center_1(2) - R*cos(linspace(goal(3) - path.beta_1, goal(3), N_2)')
            ];
        
        s_x = linspace(points(N_0, 1), points(N_0+N_1+1, 1), N_1 + 2);
        s_y = linspace(points(N_0, 2), points(N_0+N_1+1, 2), N_1 + 2);
        points(N_0+1:N_0+N_1, :) = [s_x(2:end-1)', s_y(2:end-1)'];
        
    case 4 % lsr
        center_0 = [start(1) - R*sin(start(3)), start(2) + R*cos(start(3))];
        center_1 = [goal(1) + R*sin(goal(3)), goal(2) - R*cos(goal(3))];
        points(1:N_0, :) = [ 
                center_0(1) + R*sin(linspace(start(3), start(3) + path.beta_0, N_0)'),...
                center_0(2) - R*cos(linspace(start(3), start(3) + path.beta_0, N_0)')
            ];
        points(N_0+N_1+1:N, :) = [
                center_1(1) + R*sin(-linspace(goal(3) - path.beta_1, goal(3), N_2)'),...
                center_1(2) + R*cos(-linspace(goal(3) - path.beta_1, goal(3), N_2)')
            ];
        
        s_x = linspace(points(N_0, 1), points(N_0+N_1+1, 1), N_1 + 2);
        s_y = linspace(points(N_0, 2), points(N_0+N_1+1, 2), N_1 + 2);
        points(N_0+1:N_0+N_1, :) = [s_x(2:end-1)', s_y(2:end-1)'];
        
    otherwise 
        fprintf('Error, invalid case.\n');
end

end

