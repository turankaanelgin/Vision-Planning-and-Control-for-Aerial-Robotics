function adjustedVel = adjust_velocities(velocities)
    % In case of abrupt changes, smooths the velocities by assigning
    % the previous ones
    adjustedVel = velocities;
    prevVel = velocities(1, :);
    
    for i = 2:size(velocities,1)
        currVel = adjustedVel(i, :);
        if abs(currVel(1) - prevVel(1)) > 0.5
            adjustedVel(i, 1) = prevVel(1);
        end
        if abs(currVel(2) - prevVel(2)) > 0.5
            adjustedVel(i, 2) = prevVel(2);
        end
        if abs(currVel(3) - prevVel(3)) > 0.5
            adjustedVel(i, 3) = prevVel(3);
        end
        prevVel = adjustedVel(i, :);
    end
end