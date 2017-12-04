function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
    if isempty(map)
        C = [];
    else
        M = size(points, 1);
        C = zeros(M, 1);
        for i = 1:M
            % compute the grid numbers of the point
            xGrid = ceil((points(i, 1) - map{2}(1)) / map{5});
            yGrid = ceil((points(i, 2) - map{3}(1)) / map{5});
            zGrid = ceil((points(i, 3) - map{4}(1)) / map{6});

            if xGrid <= 0
                xGrid = 1;
            end
            if yGrid <= 0
                yGrid = 1;
            end
            if zGrid <= 0
                zGrid = 1;
            end
            
            % check the particular grid from the map
            C(i) = map{1}(xGrid, yGrid, zGrid);
        end
    end
end
