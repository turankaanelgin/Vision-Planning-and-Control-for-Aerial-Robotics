function vel = convert_velocities(poses, velocities, K)
    nrFrames = size(poses, 1);
    vel = zeros(nrFrames, 3);
    
    for i = 1:nrFrames
        pose = poses(i, :);
        R = eul2rotm([pose(4), pose(5), pose(6)]);
        vel(i, :) = (R \ (K \ velocities(i, :).')).';
    end
end