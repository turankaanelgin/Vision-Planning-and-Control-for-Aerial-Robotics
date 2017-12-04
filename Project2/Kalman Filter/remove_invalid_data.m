function [filteredPoses, filteredGTSAMPoses, filteredVel, filteredGTSAMVel, ...
          filteredTimestamps] = remove_invalid_data(poses, vel, GTSAMPoses, ...
                                                       GTSAMVel, TLeftImgs)
    % Removes the 0 rows from GTSAM, iSAM2, and velocity data
    nrFrames = size(poses, 1);
    filteredPoses = zeros(nrFrames, 7);
    filteredGTSAMPoses = zeros(nrFrames, 7);
    filteredVel = zeros(nrFrames, 3);
    filteredGTSAMVel = zeros(nrFrames, 3);
    filteredTimestamps = zeros(nrFrames, 1);
    
    index = 1;
    for i = 1:nrFrames
        if any(poses(i, :)) && any(vel(i, :))
            filteredPoses(index, :) = poses(i, :);
            filteredGTSAMPoses(index, :) = GTSAMPoses(i, :);
            filteredVel(index, :) = vel(i, :);
            filteredGTSAMVel(index, :) = GTSAMVel(i, :);
            filteredTimestamps(index) = TLeftImgs(index);
            index = index + 1;
        end 
    end
    
    filteredPoses = filteredPoses(1:index-1, :);
    filteredGTSAMPoses = filteredGTSAMPoses(1:index-1, :);
    filteredVel = filteredVel(1:index-1, :);
    filteredGTSAMVel = filteredGTSAMVel(1:index-1, :);
    filteredTimestamps = filteredTimestamps(1:index-1);
end