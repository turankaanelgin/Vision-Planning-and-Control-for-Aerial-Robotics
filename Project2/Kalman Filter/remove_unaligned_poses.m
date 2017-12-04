function [filterPoses, measuredPoses, filterGTSAMPoses] = ...
                    remove_unaligned_poses(poses, AllPosesComputed, GTSAMPoses)
    % Remove the output poses which were not assigned to IMU timestamps
    nrFrames = size(poses, 1);
    filterPoses = zeros(nrFrames, 6);
    measuredPoses = zeros(nrFrames, 6);
    filterGTSAMPoses = zeros(nrFrames, 6);
    
    index = 1;
    for i = 1:nrFrames
        if any(poses(i, :))
            filterPoses(index, :) = poses(i, :);
            pose = AllPosesComputed(i, :);
            q = quat2eul([pose(4), pose(5), pose(6), pose(7)]);
            q = [q(1), q(3), q(2)];
            measuredPoses(index, :) = [pose(1), pose(2), pose(3), ...
                                       q(1), q(2), q(3)];
            pose = GTSAMPoses(i, :);
            q = quat2eul([pose(4), pose(5), pose(6), pose(7)]);
            q = [q(1), q(3), q(2)];
            filterGTSAMPoses(index, :) = [pose(1), pose(2), pose(3), ...
                                          q(1), q(2), q(3)];
            index = index + 1;
        end
    end
    
    filterPoses = filterPoses(1:index-1, :);
    measuredPoses = measuredPoses(1:index-1, :);
    filterGTSAMPoses = filterGTSAMPoses(1:index-1, :);
end