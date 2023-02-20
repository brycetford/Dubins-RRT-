function [cost] = integrate_loadfunc_dubins(start, goal, path, R, speed, loadfunc)
    candpath = generate_points(start, goal, path, R);
    
    N = size(candpath, 1);
    cost = 0;
    
    %center numerical integration
    for i = 1:N-1
        dc = (loadfunc(candpath(i,1), candpath(i,2)) + loadfunc(candpath(i+1,1), candpath(i+1,2)))/2;
        dt = norm(candpath(i, :) - candpath(i+1, :))/speed;
        cost = cost + dc*dt;
    end
end

