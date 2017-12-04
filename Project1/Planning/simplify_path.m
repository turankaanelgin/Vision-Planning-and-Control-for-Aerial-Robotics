% Simplifies the path produced by Dijkstra/A*
% Eliminates too many zigzags

function [simple_path] = simplify_path(map, path)
    nrPoints = size(path, 1);
    xy_res = map{5};
    z_res = map{6};
    
    if nrPoints == 0
        simple_path = path;
    else
        start = path(1, :);
        simple_path = zeros(nrPoints, 3);
        simple_path(1, :) = start;
        index = 2;
        currPoint = 1;
        while currPoint ~= nrPoints
            for i = currPoint+1:nrPoints
                init = path(currPoint, :);
                final = path(i, :);
                
                if final(1) > init(1)
                    xPoints = init(1):xy_res:final(1);
                else
                    xPoints = init(1):-xy_res:final(1);
                end
                
                if final(2) > init(2)
                    yPoints = init(2):xy_res:final(2);
                else
                    yPoints = init(2):-xy_res:final(2);
                end
                
                if final(3) > init(3)
                    zPoints = init(3):z_res:final(3);
                else
                    zPoints = init(3):-z_res:final(3);
                end
                
                % Divide the x-y-z intervals into equal length points
                % according to the smallest resolution
                length = max(max(size(xPoints, 2), size(yPoints, 2)), size(zPoints, 2));
                xPoints = linspace(init(1), final(1), length);
                yPoints = linspace(init(2), final(2), length);
                zPoints = linspace(init(3), final(3), length);
                set_of_points = zeros(length, 3);
                
                for k = 1:length
                    set_of_points(k, :) = [xPoints(1, k) yPoints(1, k) zPoints(1, k)];
                end
                
                % Check for collisions
                C = collide(map, set_of_points);
                nrCollisions = sum(C(:));
                if nrCollisions ~= 0
                    currPoint = i - 1;
                    simple_path(index, :) = path(currPoint, :);
                    index = index + 1;
                    break;
                end
            end
            if i == nrPoints
                currPoint = i;
                simple_path(index, :) = path(currPoint, :);
                simple_path = simple_path(1:index, :);
            end
        end
    end
end