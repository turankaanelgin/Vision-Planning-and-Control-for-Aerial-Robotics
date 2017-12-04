function [filterVelocities, filteriSAM2, filterGTSAM] = ...
           remove_unaligned_velocities(velocities, VeliSAM2, VelGTSAM)
    % Remove the output velocities which were not assigned to IMU timestamps   
    nrFrames = size(velocities, 1);
    filterVelocities = zeros(nrFrames, 3);
    filteriSAM2 = zeros(nrFrames, 3);
    filterGTSAM = zeros(nrFrames, 3);
    
    index = 1;
    for i = 1:nrFrames
        if any(velocities(i, :))
            filterVelocities(index, :) = velocities(i, :);
            vel = VeliSAM2(i, :);
            filteriSAM2(index, :) = [vel(1), vel(2), vel(3)];
            vel = VelGTSAM(i, :);
            filterGTSAM(index, :) = [vel(1), vel(2), vel(3)];
            index = index + 1;
        end
    end
    
    filterVelocities = filterVelocities(1:index-1, :);
    filteriSAM2 = filteriSAM2(1:index-1, :);
    filterGTSAM = filterGTSAM(1:index-1, :);
end