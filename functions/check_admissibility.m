function [adm] = check_admissibility(obstacles, newnode, current)
    p0 = current; %start and end for path
    d0 = newnode-p0;
    adm = 1;
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
                    adm = 0;
                    return;
                end
           end
       end
   end 
end

