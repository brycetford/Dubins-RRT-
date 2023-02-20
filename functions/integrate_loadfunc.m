function [cost] = integrate_loadfunc(start, goal, speed, loadfunc) 
    N = max(4, ceil(norm(start - goal)));
    cost = 0;
    
    candpath = [linspace(start(1), goal(1), N)', linspace(start(2), goal(2), N)'];
    
    %center numerical integration
    for i = 1:N-1
        dc = (loadfunc(candpath(i,1), candpath(i,2)) + loadfunc(candpath(i+1,1), candpath(i+1,2)))/2;
        dt = norm(candpath(i, :) - candpath(i+1, :))/speed;
        cost = cost + dc*dt;
    end
end
