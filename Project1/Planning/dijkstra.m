function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
    if nargin < 4
        astar = false;
    end

    path_exists = false;
    [xDim, yDim, zDim] = size(map{1});
    xy_res = map{5};
    z_res = map{6};
    % compute the grid numbers of the start point
    startXGrid = ceil((start(1) - map{2}(1)) / map{5});
    startYGrid = ceil((start(2) - map{3}(1)) / map{5});
    startZGrid = ceil((start(3) - map{4}(1)) / map{6});

    if startXGrid <= 0
        startXGrid = 1;
    end
    if startYGrid <= 0
        startYGrid = 1;
    end
    if startZGrid <= 0
        startZGrid = 1;
    end

    % compute the grid numbers of the goal point
    goalXGrid = ceil((goal(1) - map{2}(1)) / map{5});
    goalYGrid = ceil((goal(2) - map{3}(1)) / map{5});
    goalZGrid = ceil((goal(3) - map{4}(1)) / map{6});

    if goalXGrid <= 0
        goalXGrid = 1;
    end
    if goalYGrid <= 0
        goalYGrid = 1;
    end
    if goalZGrid <= 0
        goalZGrid = 1;
    end

    % define a lookup table for parents of the nodes and initialize the
    % start point's parent as empty
    parentNodes = containers.Map;
    parentNodes(sprintf('%d#%d#%d', startXGrid, startYGrid, startZGrid)) = ...
                                                                       'empty';
    % the map which keeps the distances of each grid
    % initialize start point's distance as one, others as infinity
    distanceMap = inf * ones(xDim, yDim, zDim);
    distanceMap(startXGrid, startYGrid, startZGrid) = 0;

    % visited map keeps if a node is visited or not
    % at first, all of it is zero (unvisited)
    visitedMap = zeros(xDim, yDim, zDim);
    num_expanded = 0;
    nrUnvisited = xDim * yDim * zDim;

    % if astar is used, heuristic map keeps distance + (distance to goal)
    % else, it is the same as the distance map
    if astar
        heuristicMap = inf * ones(xDim, yDim, zDim);
        heuristicMap(startXGrid, startYGrid, startZGrid) = 1;
    else
        heuristicMap = distanceMap;
    end

    while nrUnvisited > 0
        % get the node with minimum heuristic
        [val, index] = min(heuristicMap(:)); 
        if val == inf % if all of them is inf, that means there is no path
            break;
        end

        % get the grid location
        [currX, currY, currZ] = ind2sub(size(heuristicMap), index);
        % mark the note as visited
        heuristicMap(currX, currY, currZ) = inf; 
        visitedMap(currX, currY, currZ) = 1;
        if map{1}(currX, currY, currZ) % if it is an obstacle, continue
            continue;
        end

        % if it is the goal, we are finished
        if currX == goalXGrid && currY == goalYGrid && currZ == goalZGrid 
            path_exists = true;
            break;
        end
        currDist = distanceMap(currX, currY, currZ);

        % for each of its 26 neighbors
        for x = -1:1
            for y = -1:1
                for z = -1:1
                    if x == 0 && y == 0 && z == 0
                        continue;
                    else
                        % if the neighbor is in the borders
                        if currX+x >= 1 && currY+y >= 1 && currZ+z >= 1 && ...
                           currX+x <= xDim && currY+y <= yDim && currZ+z <= zDim
                            % if the neighbor is not an obstacle, and it has
                            % not been visited before
                            if ~map{1}(currX+x, currY+y, currZ+z) && ...
                                    ~visitedMap(currX+x, currY+y, currZ+z)
                                distanceToCurr = sqrt((x * xy_res)^2 + ...
                                          (y * xy_res)^2 + (z * z_res)^2);
                                if distanceMap(currX+x, currY+y, currZ+z) > ...
                                                currDist + distanceToCurr
                                    distanceMap(currX+x, currY+y, currZ+z) = ...
                                                currDist + distanceToCurr;
                                    heuristicMap(currX+x, currY+y, currZ+z) = ...
                                                currDist + distanceToCurr;

                                    % if astar is used, update the heuristic
                                    % value
                                    if astar
                                        distanceToGoal = sqrt(...
                                          ((currX+x - goalXGrid)*xy_res)^2 + ...
                                          ((currY+y - goalYGrid)*xy_res)^2 + ...
                                          ((currZ+z - goalZGrid)*z_res)^2);
                                        heuristicMap(currX+x, currY+y, currZ+z) = ...
                                            heuristicMap(currX+x, currY+y, currZ+z) + ...
                                                                distanceToGoal;
                                    end
                                    % assign neighbor's parent as current node
                                    parentNodes(sprintf('%d#%d#%d', currX+x, currY+y, currZ+z)) = ...
                                                sprintf('%d#%d#%d', currX, currY, currZ);
                                end
                            end
                        end
                    end
                end
            end
        end

        num_expanded = num_expanded + 1;
        nrUnvisited = nrUnvisited - 1;
    end

    if path_exists 
        path = zeros(num_expanded, 3);
        index = 1;
        current = [goalXGrid, goalYGrid, goalZGrid]; % go backwards
        while true
            currX = map{2}(current(1)) + xy_res / 2;
            currY = map{3}(current(2)) + xy_res / 2;
            currZ = map{4}(current(3)) + z_res / 2;
            path(index, :) = [currX currY currZ];
            index = index + 1;
            % switch to its parent
            parent = parentNodes(sprintf('%d#%d#%d', current(1), ...
                                 current(2), current(3)));
            if strcmp(parent, 'empty') % until its parent is empty
                break;
            else
                coord = strsplit(parent, '#');
                current = [str2double(coord(1)), str2double(coord(2)), ...
                           str2double(coord(3))];
            end
        end
        path = path(1:index-1, :);
        path = flipud(path); % flip the path
        path(1, :) = [start(1) start(2) start(3)];
        path(index-1, :) = [goal(1) goal(2) goal(3)];
    else
        path = [];
    end
end
