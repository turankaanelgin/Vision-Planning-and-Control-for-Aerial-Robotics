function adjustedPoses = adjust_poses(poses)
    adjustedPoses = poses;
    prevPose = poses(1, :);
    
    for i = 2:size(poses,1)
        currPose = adjustedPoses(i, :);
        if abs(currPose(1) - prevPose(1)) > 1.5
            adjustedPoses(i, 1) = prevPose(1);
        end
        if abs(currPose(2) - prevPose(2)) > 1.5
            adjustedPoses(i, 2) = prevPose(2);
        end
        if abs(currPose(3) - prevPose(3)) > 1.5
            adjustedPoses(i, 3) = prevPose(3);
        end
        prevPose = adjustedPoses(i, :);
    end
end