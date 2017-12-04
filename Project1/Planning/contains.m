% Determines if the point is inside the obstacle
function c = contains(gridX0, gridY0, gridZ0, gridX1, gridY1, gridZ1, ...
                      obsX0, obsY0, obsZ0, obsX1, obsY1, obsZ1)
    xOverlap = 0;
    yOverlap = 0;
    zOverlap = 0;
    
    if gridX0 <= obsX0 && obsX0 <= gridX1
        xOverlap = 1;
    elseif gridX0 <= obsX1 && obsX1 <= gridX1
        xOverlap = 1;
    elseif obsX0 <= gridX0 && gridX0 <= obsX1
        xOverlap = 1;
    elseif obsX0 <= gridX1 && gridX1 <= obsX1
        xOverlap = 1;
    end
    
    if gridY0 <= obsY0 && obsY0 <= gridY1
        yOverlap = 1;
    elseif gridY0 <= obsY1 && obsY1 <= gridY1
        yOverlap = 1;
    elseif obsY0 <= gridY0 && gridY0 <= obsY1
        yOverlap = 1;
    elseif obsY0 <= gridY1 && gridY1 <= obsY1
        yOverlap = 1;
    end
    
    if gridZ0 <= obsZ0 && obsZ0 <= gridZ1
        zOverlap = 1;
    elseif gridZ0 <= obsZ1 && obsZ1 <= gridZ1
        zOverlap = 1;
    elseif obsZ0 <= gridZ0 && gridZ0 <= obsZ1
        zOverlap = 1;
    elseif obsZ0 <= gridZ1 && gridZ1 <= obsZ1
        zOverlap = 1;
    end
    
    c = xOverlap && yOverlap && zOverlap;
end