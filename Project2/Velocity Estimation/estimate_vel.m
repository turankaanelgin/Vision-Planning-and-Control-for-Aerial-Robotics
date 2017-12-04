function [vel, omg, AllPosesComputed] = estimate_vel(DetAll, LandMarksComputed, ...
                                            K, TagSize, LeftImgs, TLeftImgs)
    % get poses from ISAM
    AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, 0, 0,...
                                              0, 0, 0, LandMarksComputed);
    nrFrames = size(AllPosesComputed, 1);
    corners = cell(nrFrames, 1);
    trackedCorners = cell(nrFrames, 1);
    sparseOpticalFlow = cell(nrFrames - 1, 1);
    derivatives = cell(nrFrames - 1, 1);
    centerX = K(1, 3);
    centerY = K(2, 3);
    
    % detect corners
    for i = 1:nrFrames
        LeftImgs{i} = rgb2gray(LeftImgs{i});
        C = detectHarrisFeatures(LeftImgs{i});
        corners{i} = C.Location;
    end
    disp('Corners were detected...');
    
    % compute constant delta T
    deltaT = 0;
    for i = 2:nrFrames
        deltaT = deltaT + (TLeftImgs(i) - TLeftImgs(i-1));
    end
    deltaT = deltaT / (nrFrames-1);
    
    % track corners
    prevPoints = corners{1};
    kltTracker = vision.PointTracker();
    initialize(kltTracker, prevPoints, LeftImgs{1});
    for i = 2:nrFrames
        [points, validIdx] = step(kltTracker, LeftImgs{i});
        matchedPrevPoints = prevPoints(validIdx, :);
        matchedPoints = points(validIdx, :);
        trackedCorners{i - 1} = matchedPrevPoints;
        sparseOpticalFlow{i - 1} = matchedPoints - matchedPrevPoints;
%         deltaT = TLeftImgs(i) - TLeftImgs(i-1);
        derivatives{i - 1} = sparseOpticalFlow{i - 1} / deltaT;
        prevPoints = matchedPoints;
        setPoints(kltTracker, prevPoints);
        % initialize once in 10 frame
        if mod(i, 10) == 1
            kltTracker = vision.PointTracker();
            prevPoints = corners{i};
            initialize(kltTracker, prevPoints, LeftImgs{i});
        end
    end
    trackedCorners{i} = matchedPoints;
    corners = trackedCorners;
    disp('Sparse optical flows were estimated...');
    
    vel_c = zeros(3, nrFrames);
    omg_c = zeros(3, nrFrames);
    
    for i = 2:nrFrames
        pose = AllPosesComputed(i, :);
        if ~any(pose, 2)
            continue;
        end
        
        % compute corner depths from pitch angle and height
        R = QuatToRot([pose(4), pose(5), pose(6), pose(7)]).';
        [~,theta,~] = RotToRPY_ZXY(R);
        Z = abs(pose(3) / cos(theta));
        C = corners{i-1};
        
        if size(C, 1) < 3
            continue;
        end
        
        % get inliers from ransac
        flowVectors = derivatives{i-1};
        inlierIdx = ransac(C, flowVectors, Z, 100, 100, 10, K);
        nrInliers = length(inlierIdx);
        
        if nrInliers < 3
            continue;
        end
        
        % compute velocities in camera frame
        flows = zeros(2*nrInliers, 1);
        f = zeros(2*nrInliers, 6);
        
        index = 1;
        for j = 1:nrInliers
            flows(index:index+1) = flowVectors(inlierIdx(j), :)';
            point = C(inlierIdx(j), :);
            x = point(1) - centerX;
            y = point(2) - centerY;
            f1 = K(1, 1) * [-1/Z, 0, x/Z, x*y, -(x^2+1), y];
            f2 = K(2, 2) * [0, -1/Z, y/Z, y^2+1, -x*y, -x];
            f(index, :) = f1;
            f(index+1, :) = f2;
            index = index + 2;
        end
       
        velocities = f \ flows;
        vel_c(:, i) = velocities(1:3);
        omg_c(:, i) = velocities(4:6);
        fprintf('Frame %d was processed\n', i);
    end
    
    % convert velocities into world frame
    vel = zeros(nrFrames, 3);
    omg = zeros(nrFrames, 3);
    for i = 2:nrFrames
        pose = AllPosesComputed(i, :);
        Rc = QuatToRot([pose(4), pose(5), pose(6), pose(7)]);
        R = Rc.';
        vel(i, :) = (R \ vel_c(:, i)).';
        omg(i, :) = (R \ omg_c(:, i)).';
    end
    
    filter = 1/10 * ones(1, 5);
    for i = 1:3
        vel(:, i) = conv(vel(:, i).', filter, 'same');
        omg(:, i) = conv(vel(:, i).', filter, 'same');
    end
end