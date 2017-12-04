function map  = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
    fileID = fopen(filename);
    fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
    fclose(fileID);

    map = cell(7, 1);
    nrLines = size(fileDat{1, 1}, 1);
    boundaryIndex = 0;
    blockIndices = 0;
    
    for i = 1:nrLines
        if strcmp(fileDat{1, 1}{i}, 'boundary')
            boundaryIndex = i;
            blockIndices = [1:1:i-1, i+1:1:nrLines];
            break;
        end
    end
    
    bdryXMin = fileDat{1, 2}(boundaryIndex);
    bdryYMin = fileDat{1, 3}(boundaryIndex);
    bdryZMin = fileDat{1, 4}(boundaryIndex);
    bdryXMax = fileDat{1, 5}(boundaryIndex);
    bdryYMax = fileDat{1, 6}(boundaryIndex);
    bdryZMax = fileDat{1, 7}(boundaryIndex);
    
    nrXCells = ceil((bdryXMax - bdryXMin) / xy_res);
    nrYCells = ceil((bdryYMax - bdryYMin) / xy_res);
    nrZCells = ceil((bdryZMax - bdryZMin) / z_res);
    nrObstacles = size(fileDat{1}, 1) - 1;

    map{1} = zeros(nrXCells, nrYCells, nrZCells);
    map{2} = zeros(nrXCells + 1, 1);
    map{3} = zeros(nrYCells + 1, 1);
    map{4} = zeros(nrZCells + 1, 1);

    % discretize the coordinate values including the start 
    % and end points plus the boundaries
    x = bdryXMin;
    index = 1;
    while x <= bdryXMax
        map{2}(index) = x;
        index = index + 1;
        x = x + xy_res;
    end
    if index == nrXCells + 1
        map{2}(index) = bdryXMax;
    end

    y = bdryYMin;
    index = 1;
    while y <= bdryYMax
        map{3}(index) = y;
        index = index + 1;
        y = y + xy_res;
    end
    if index == nrYCells + 1
        map{3}(index) = bdryYMax;
    end

    z = bdryZMin;
    index = 1;
    while z <= bdryZMax
        map{4}(index) = z;
        index = index + 1;
        z = z + z_res;
    end
    if index == nrZCells + 1
        map{4}(index) = bdryZMax;
    end

    % for each grid, if its distance to any one of the obstacles is
    % smaller than margin, mark it as filled
    % the distance is computed w.r.t to the center of the grid
    for x = 1:nrXCells
        for y = 1:nrYCells
            for z = 1:nrZCells
                % compute the center of the grid
    %             gridX = map{2}(x) + xy_res / 2;
    %             gridY = map{3}(y) + xy_res / 2;
    %             gridZ = map{4}(z) + z_res / 2;
                gridX0 = map{2}(x);
                gridY0 = map{3}(y);
                gridZ0 = map{4}(z);
                gridX1 = map{2}(x+1);
                gridY1 = map{3}(y+1);
                gridZ1 = map{4}(z+1);

                % for each obstacle, compute the distance
                % if any of them collides with that, the mark it as filled
                % and break
                for i = 1:nrObstacles
                    obsx0 = fileDat{1, 2}(blockIndices(i));
                    obsy0 = fileDat{1, 3}(blockIndices(i));
                    obsz0 = fileDat{1, 4}(blockIndices(i));
                    obsx1 = fileDat{1, 5}(blockIndices(i));
                    obsy1 = fileDat{1, 6}(blockIndices(i));
                    obsz1 = fileDat{1, 7}(blockIndices(i));

                    map{1}(x, y, z) = contains(gridX0, gridY0, gridZ0, ...
                                               gridX1, gridY1, gridZ1, ...
                                      obsx0 - margin, obsy0 - margin, ...
                                      obsz0 - margin, obsx1 + margin, ...
                                      obsy1 + margin, obsz1 + margin);
                    if map{1}(x, y, z)
                        break;
                    end
                end
            end
        end
    end

    map{5} = xy_res;
    map{6} = z_res;

    % keep obstacle data for plotting the path
    map{7} = zeros(nrObstacles, 9);
    for i = 1:nrObstacles
        map{7}(i, 1) = fileDat{1, 2}(blockIndices(i));
        map{7}(i, 2) = fileDat{1, 3}(blockIndices(i));
        map{7}(i, 3) = fileDat{1, 4}(blockIndices(i));
        map{7}(i, 4) = fileDat{1, 5}(blockIndices(i));
        map{7}(i, 5) = fileDat{1, 6}(blockIndices(i));
        map{7}(i, 6) = fileDat{1, 7}(blockIndices(i)); 
        map{7}(i, 7) = fileDat{1, 8}(blockIndices(i));
        map{7}(i, 8) = fileDat{1, 9}(blockIndices(i));
        map{7}(i, 9) = fileDat{1, 10}(blockIndices(i));
    end
end
