function [adm] = check_admissibility_dubins(start, goal, path, R, obstacles)
    % Check for collisions for given dubins path
    adm = true;

    switch path.type
        case 1 % lsl
            center_0 = [start(1) - R*sin(start(3)), start(2) + R*cos(start(3))]; % left center
            center_1 = [goal(1) - R*sin(goal(3)), goal(2) + R*cos(goal(3))]; % left center
            
            % turn left intermediate point
            inter_0 = [
                    center_0(1) + R*sin(start(3) + path.beta_0),...
                    center_0(2) - R*cos(start(3) + path.beta_0),...
                    start(3) + path.beta_0
                ];
            % turn left intermediate point
            inter_1 = [
                    center_1(1) + R*sin(goal(3) - path.beta_1),...
                    center_1(2) - R*cos(goal(3) - path.beta_1),...
                    goal(3) - path.beta_1
                ];
            
            % check each primitive in the path individually
            adm = check_s(inter_0, inter_1, obstacles) & check_l(start, inter_0, R, obstacles) & check_l(inter_1, goal, R, obstacles);
            return;
            
        case 2 % rsr
            center_0 = [start(1) + R*sin(start(3)), start(2) - R*cos(start(3))]; % right center
            center_1 = [goal(1) + R*sin(goal(3)), goal(2) - R*cos(goal(3))]; % right center
            
            % turn right intermediate point
            inter_0 = [
                    center_0(1) + R*sin(-(start(3) + path.beta_0)),...
                    center_0(2) + R*cos(-(start(3) + path.beta_0)),...
                    start(3) + path.beta_0
                ];
            % turn right intermediate point
            inter_1 = [
                    center_1(1) + R*sin(-(goal(3) - path.beta_1)),...
                    center_1(2) + R*cos(-(goal(3) - path.beta_1)),...
                    goal(3) - path.beta_1
                ];
            
            % check each primitive in the path individually
            adm = check_s(inter_0, inter_1, obstacles) & check_r(start, inter_0, R, obstacles) & check_r(inter_1, goal, R, obstacles);
            return;
            
        case 3 % rsl
            center_0 = [start(1) + R*sin(start(3)), start(2) - R*cos(start(3))]; % right center
            center_1 = [goal(1) - R*sin(goal(3)), goal(2) + R*cos(goal(3))]; % left center
            
            % turn right intermediate point
            inter_0 = [
                    center_0(1) + R*sin(-(start(3) + path.beta_0)),...
                    center_0(2) + R*cos(-(start(3) + path.beta_0)),...
                    start(3) + path.beta_0
                ];
            % turn left intermediate point
            inter_1 = [
                    center_1(1) + R*sin(goal(3) - path.beta_1),...
                    center_1(2) - R*cos(goal(3) - path.beta_1),...
                    goal(3) - path.beta_1
                ];
            
            % check each primitive in the path individually
            adm = check_s(inter_0, inter_1, obstacles) & check_r(start, inter_0, R, obstacles) & check_l(inter_1, goal, R, obstacles);
            return;
            
        case 4 % lsr
            center_0 = [start(1) - R*sin(start(3)), start(2) + R*cos(start(3))]; % left center
            center_1 = [goal(1) + R*sin(goal(3)), goal(2) - R*cos(goal(3))]; % right center
            
            % turn left intermediate point
            inter_0 = [
                    center_0(1) + R*sin(start(3) + path.beta_0),...
                    center_0(2) - R*cos(start(3) + path.beta_0),...
                    start(3) + path.beta_0
                ];
            % turn right intermediate point
            inter_1 = [
                    center_1(1) + R*sin(-(goal(3) - path.beta_1)),...
                    center_1(2) + R*cos(-(goal(3) - path.beta_1)),...
                    goal(3) - path.beta_1
                ];
            
            % check each primitive in the path individually
            adm = check_s(inter_0, inter_1, obstacles) & check_l(start, inter_0, R, obstacles) & check_r(inter_1, goal, R, obstacles);
            return;
            
        otherwise
            fprintf('Error, invalid case.\n');
    end
end

function [adm] = check_s(start, goal, obstacles)
    adm = true;
    p0 = start(1:2); %start and end for path
    d0 = goal(1:2)-p0;
    for i = 1:obstacles.number
        %for each obstacle check the path with each edge
        verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
        for j = 1:size(verts, 1)-1
            p1 = verts(j,:); %start and endpoint for edge
            d1 = verts(j+1,:)-p1;
            %checking for an intersection
            denom = d0(1)*d1(2) - d1(1)*d0(2); % the lines intersect if this product is not zero
            if denom ~= 0
                delt = p1 - p0;
                s = (delt(1)*d1(2) - d1(1)*delt(2))/denom;
                t = (delt(1)*d0(2) - d0(1)*delt(2))/denom;
                %if the lines intersect andd the point is on both
                %the obstacle line and the path return and set adm
                %to false
                if (s>=0 && s<=1) && (t>=0 && t<=1) 
                    adm = false;
                    return;
                end
            end
        end
    end
end

function [adm] = check_r(start, goal, R, obstacles)
    adm = true;
    u = [sin(start(3)) -cos(start(3))]*R; %vector ccw perpendicular to current pose * turnrad
    p1 = start(1:2);
    d1 = goal(1:2)-p1;
    c = p1 + u; % center of the turn arc
    for i = 1:obstacles.number
        %for each obstacle check the path with each edge
        verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
        for j = 1:size(verts, 1)-1
            p2 = verts(j,:); %start and endpoint for edge
            d2 = verts(j+1,:)-p2;
            % checking for an intersection
            delta = p2 - c;
            cond = (d2*delta')^2 - norm(d2)^2 * (norm(delta)^2 - R^2); %if roots are not complex the line and the circle intersect
            if cond >= 0 %find points of intersection and check if they lie on the arc, if one does, set adm to false and return
                t1 = (-d2*delta' + sqrt(cond))/norm(d2)^2;
                t2 = (-d2*delta' - sqrt(cond))/norm(d2)^2;
                if t1 >= 0 && t1 <= 1
                    p = p2 + t1*d2;
                    B = p-p1; %because turn down is going cw need to swap bounds of the arc
                    A = d1;
                    if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                        adm = false;
                        return;
                    end
                end
                if t2 >= 0 && t2 <= 1 
                    p = p2 + t2*d2;
                    B = p-p1; %because turn down is going cw need to swap bounds of the arc
                    A = d1;
                    if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                        adm = false;
                        return;
                    end
                end 
            end
        end
    end
end

function [adm] = check_l(start, goal, R, obstacles)
    adm = true;
    u = [-sin(start(3)) cos(start(3))]*R; %vector ccw perpendicular to current pose * turnrad
    p1 = start(1:2); 
    d1 = goal(1:2)-p1;
    c = p1 + u; % center of the turn arc
    for i = 1:obstacles.number
        %for each obstacle check the path with each edge
        verts = [obstacles.vertices{i}; obstacles.vertices{i}(1,:)]; %array of verices where the first row = last row to close the object
        for j = 1:size(verts, 1)-1
            p2 = verts(j,:); %start and endpoint for edge
            d2 = verts(j+1,:)-p2;
            % checking for the intersection
            delta = p2 - c;
            cond = (d2*delta')^2 - norm(d2)^2 * (norm(delta)^2 - R^2); %if roots are not complex the line and the circle intersect
            if cond >= 0 %find points of intersection and check if they lie on the arc, if one does, set adm to false and return
                t1 = (-d2*delta' + sqrt(cond))/norm(d2)^2;
                t2 = (-d2*delta' - sqrt(cond))/norm(d2)^2;
                if t1 >= 0 && t1 <= 1
                    p = p2 + t1*d2;
                    A = p-p1;
                    B = d1;
                    if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                        adm = false;
                        return;
                    end
                end
                if t2 >= 0 && t2 <= 1 
                    p = p2 + t2*d2;
                    A = p-p1;
                    B = d1;
                    if (A(1)*B(2) - B(1)*A(2)) >= 0 %check if the point lies on the arc
                        adm = false;
                        return;
                    end
                end 
            end
        end
    end 
end