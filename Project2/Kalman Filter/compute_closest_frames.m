function framesPerIMU = compute_closest_frames(IMU, filteredTimestamps, ...
                                               TLeftImgs)
    % Compute the closest frame to each IMU timestamp
    nrIMUData = length(filteredTimestamps);
    nrFrames = size(TLeftImgs, 1);
    framesPerIMU = zeros(nrIMUData, 1);
    
    for i = 1:nrIMUData
        tIMU = IMU(filteredTimestamps(i), 11);
        diff = zeros(nrFrames, 1);
        
        for j = 1:nrFrames
            tImg = TLeftImgs(j);
            diff(j) = abs(tImg - tIMU);
        end
        
        [~, framesPerIMU(i)] = min(diff);
    end
end