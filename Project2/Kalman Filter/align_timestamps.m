function filteredTimestamps = align_timestamps(TLeftImgs, IMU)
    % Aligns the timestamps of IMU with frame timestamps
    nrFrames = size(TLeftImgs, 1);
    nrIMUData = size(IMU, 1);
    filteredTimestamps = zeros(1, nrIMUData);
    diffThreshold = 0.00211; % tuned
    index = 1;
    
    for i = 1:nrIMUData
        % For each IMU timestamp, find the frame with the timestamp
        % which has the smallest distance 
        minDiff = inf;
        tIMU = IMU(i, 11);
        
        for j = 1:nrFrames
            tImage = TLeftImgs(j);
            if abs(tImage - tIMU) < minDiff
                minDiff = abs(tImage - tIMU);
            end
        end
        
        % Check if the smallest distance is less than the threshold
        if minDiff <= diffThreshold
            filteredTimestamps(1, index) = i;
            index = index + 1;
        end
    end
    
    filteredTimestamps = filteredTimestamps(1:index-1);
end