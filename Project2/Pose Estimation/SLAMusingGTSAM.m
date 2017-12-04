function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, Mode)
% For Input and Output specifications refer to the project pdf

import gtsam.*
warning off;
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

% create lookup tables for symbols
tagSymbols = containers.Map('KeyType', 'double', 'ValueType', 'any');
cameraSymbols = containers.Map('KeyType', 'double', 'ValueType', 'any');
% world coordinates of tags
worldCoordinates = containers.Map('KeyType', 'double', 'ValueType', 'any');
nrFrames = size(DetAll, 2);
cameraPoses = cell(nrFrames, 2);

nrValidFrames = 0;
for i = 1:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    if nrTags ~= 0
        nrValidFrames = nrValidFrames + 1;
    end
end

DetAllReal = cell(nrValidFrames, 1);
index = 1;
for i = 1:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    if nrTags ~= 0
        DetAllReal{index} = frame;
        index = index + 1;
    end
end

nrFrames = nrValidFrames;
DetAll = DetAllReal;

firstFrame = 0;
for i = 1:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    for j = 1:nrTags
        tagId = frame(j, 1);
        if ~isKey(tagSymbols, tagId)
            tagSymbols(tagId) = [symbol('a', tagId), symbol('b', tagId), ...
                                 symbol('c', tagId), symbol('d', tagId)];
        end
        if tagId == 10 && firstFrame == 0
            firstFrame = i;
        end
    end
    cameraSymbols(i) = symbol('x', i);
end

% estimate rotation and translation matrices using homography
cameraParams = cameraParameters('IntrinsicMatrix', K.');
worldCoordinates(10) = [0 0 TagSize 0 TagSize TagSize 0 TagSize];
for i = firstFrame:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    imagePoints = zeros(4*nrTags, 2);
    worldPoints = zeros(4*nrTags, 3);
    tagCnt = 1;
    % get image and world coordinates of the tags so far
    for j = 1:nrTags
        tag = frame(j, :);
        if isKey(worldCoordinates, tag(1)) 
            wc = worldCoordinates(tag(1));
            imagePoints(tagCnt, :) = [tag(2) tag(3)];
            worldPoints(tagCnt, :) = [wc(1) wc(2) 0];
            imagePoints(tagCnt+1, :) = [tag(4) tag(5)];
            worldPoints(tagCnt+1, :) = [wc(3) wc(4) 0];
            imagePoints(tagCnt+2, :) = [tag(6) tag(7)];
            worldPoints(tagCnt+2, :) = [wc(5) wc(6) 0];
            imagePoints(tagCnt+3, :) = [tag(8) tag(9)];
            worldPoints(tagCnt+3, :) = [wc(7) wc(8) 0];
            tagCnt = tagCnt + 4;
        end  
    end
    imagePoints = imagePoints(1:tagCnt-1, :);
    worldPoints = worldPoints(1:tagCnt-1, :);
    
    % estimate camera pose
    try
        [orientation, location] = estimateWorldCameraPose(imagePoints, ...
                                    worldPoints, cameraParams, ...
                                    'MaxNumTrials', 500, 'Confidence', 99);
    catch
        continue;
    end

    % estimate extrinsics
    [R, T] = cameraPoseToExtrinsics(orientation, location);
    imageCoordinates = zeros(4*nrTags, 2);
    tagIds = zeros(4*nrTags, 1);
    tagCnt = 1;
    for j = 1:nrTags
        tag = frame(j, :);
        if ~isKey(worldCoordinates, tag(1))
            imageCoordinates(tagCnt, :) = [tag(2) tag(3)];
            imageCoordinates(tagCnt+1, :) = [tag(4) tag(5)];
            imageCoordinates(tagCnt+2, :) = [tag(6) tag(7)];
            imageCoordinates(tagCnt+3, :) = [tag(8) tag(9)];
            tagIds(tagCnt) = tag(1);
            tagCnt = tagCnt + 4;
        end
    end
   
    worldPoints = pointsToWorld(cameraParams, R, T, imageCoordinates);
    for j = 1:4:tagCnt
        worldPoints1 = worldPoints(j, :);
        worldPoints2 = worldPoints(j+1, :);
        worldPoints3 = worldPoints(j+2, :);
        worldPoints4 = worldPoints(j+3, :);
        worldCoordinates(tagIds(j)) = [worldPoints1(1) worldPoints1(2) ...
            worldPoints2(1) worldPoints2(2) worldPoints3(1) worldPoints3(2) ...
            worldPoints4(1) worldPoints4(2)];
    end
end

for i = firstFrame-1:-1:1
    frame = DetAll{i};
    nrTags = size(frame, 1);
    imagePoints = zeros(4*nrTags, 2);
    worldPoints = zeros(4*nrTags, 3);
    tagCnt = 1;
    % get image and world coordinates of the tags so far
    for j = 1:nrTags
        tag = frame(j, :);
        if isKey(worldCoordinates, tag(1)) 
            wc = worldCoordinates(tag(1));
            imagePoints(tagCnt, :) = [tag(2) tag(3)];
            worldPoints(tagCnt, :) = [wc(1) wc(2) 0];
            imagePoints(tagCnt+1, :) = [tag(4) tag(5)];
            worldPoints(tagCnt+1, :) = [wc(3) wc(4) 0];
            imagePoints(tagCnt+2, :) = [tag(6) tag(7)];
            worldPoints(tagCnt+2, :) = [wc(5) wc(6) 0];
            imagePoints(tagCnt+3, :) = [tag(8) tag(9)];
            worldPoints(tagCnt+3, :) = [wc(7) wc(8) 0];
            tagCnt = tagCnt + 4;
        end  
    end
    imagePoints = imagePoints(1:tagCnt-1, :);
    worldPoints = worldPoints(1:tagCnt-1, :);
    
    % estimate camera pose
    try
        [orientation, location] = estimateWorldCameraPose(imagePoints, ...
                                    worldPoints, cameraParams, ...
                                    'MaxNumTrials', 500, 'Confidence', 99);
    catch
        continue;
    end
    
    % estimate extrinsics
    [R, T] = cameraPoseToExtrinsics(orientation, location);
    imageCoordinates = zeros(4*nrTags, 2);
    tagIds = zeros(4*nrTags, 1);
    tagCnt = 1;
    for j = 1:nrTags
        tag = frame(j, :);
        if ~isKey(worldCoordinates, tag(1))
            imageCoordinates(tagCnt, :) = [tag(2) tag(3)];
            imageCoordinates(tagCnt+1, :) = [tag(4) tag(5)];
            imageCoordinates(tagCnt+2, :) = [tag(6) tag(7)];
            imageCoordinates(tagCnt+3, :) = [tag(8) tag(9)];
            tagIds(tagCnt) = tag(1);
            tagCnt = tagCnt + 4;
        end
    end
   
    worldPoints = pointsToWorld(cameraParams, R, T, imageCoordinates);
    for j = 1:4:tagCnt
        worldPoints1 = worldPoints(j, :);
        worldPoints2 = worldPoints(j+1, :);
        worldPoints3 = worldPoints(j+2, :);
        worldPoints4 = worldPoints(j+3, :);
        worldCoordinates(tagIds(j)) = [worldPoints1(1) worldPoints1(2) ...
            worldPoints2(1) worldPoints2(2) worldPoints3(1) worldPoints3(2) ...
            worldPoints4(1) worldPoints4(2)];
    end
end

for i = 1:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    imagePoints = zeros(4*nrTags, 2);
    worldPoints = zeros(4*nrTags, 3);
    tagCnt = 1;
    for j = 1:nrTags
        tag = frame(j, :);
        imagePoints(tagCnt, :) = [tag(2) tag(3)];
        imagePoints(tagCnt+1, :) = [tag(4) tag(5)];
        imagePoints(tagCnt+2, :) = [tag(6) tag(7)];
        imagePoints(tagCnt+3, :) = [tag(8) tag(9)];
        wc = worldCoordinates(tag(1));
        worldPoints(tagCnt, :) = [wc(1) wc(2) 0];
        worldPoints(tagCnt+1, :) = [wc(3) wc(4) 0];
        worldPoints(tagCnt+2, :) = [wc(5) wc(6) 0];
        worldPoints(tagCnt+3, :) = [wc(7) wc(8) 0];
        tagCnt = tagCnt + 4;
    end
    try
        [orientation, location] = estimateWorldCameraPose(imagePoints, ...
                                        worldPoints, cameraParams, ...
                                    'MaxNumTrials', 500, 'Confidence', 99);
    catch
        orientation = [0 0 0; 0 0 0; 0 0 0];
        location = [0 0 0];
    end
    cameraPoses{i, 1} = orientation;
    cameraPoses{i, 2} = location;
end
disp('Homographies were estimated...');

% add prior factors
graph = NonlinearFactorGraph;
priorMean1 = Point3(0, 0, 0);
priorMean2 = Point3(TagSize, 0, 0);
priorMean3 = Point3(TagSize, TagSize, 0);
priorMean4 = Point3(0, TagSize, 0);
pointPriorNoise  = noiseModel.Isotropic.Sigma(3, 0.01);
symbols = tagSymbols(10);
graph.add(PriorFactorPoint3(symbols(1), priorMean1, pointPriorNoise));
graph.add(PriorFactorPoint3(symbols(2), priorMean2, pointPriorNoise));
graph.add(PriorFactorPoint3(symbols(3), priorMean3, pointPriorNoise));
graph.add(PriorFactorPoint3(symbols(4), priorMean4, pointPriorNoise));

posePriorNoise  = noiseModel.Diagonal.Sigmas([0.1 0.1 0.1 0.3 0.3 0.3]');
priorPoint = cameraPoses{1, 2};
priorPoint = Point3(priorPoint(1), priorPoint(2), priorPoint(3));
posePriorMean = Pose3(Rot3(cameraPoses{1, 1}), priorPoint); 
graph.add(PriorFactorPose3(cameraSymbols(1), posePriorMean, posePriorNoise));

projectionNoise = noiseModel.Isotropic.Sigma(2, 0.5);
distanceNoise = noiseModel.Isotropic.Sigma(3, 0.5);
poseNoise = noiseModel.Diagonal.Sigmas([0.1 0.1 0.1 0.3 0.3 0.3]');
k = Cal3_S2(K(1,1), K(2,2), K(1,2), K(1,3), K(2,3));
for i = 1:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    poseKey = cameraSymbols(i);
    for j = 1:nrTags
        tag = frame(j, :);
        
        % add projection factors
        measured1 = Point2(tag(2), tag(3));
        measured2 = Point2(tag(4), tag(5));
        measured3 = Point2(tag(6), tag(7));
        measured4 = Point2(tag(8), tag(9));
        tagSym = tagSymbols(tag(1));
        pointKey1 = tagSym(1);
        pointKey2 = tagSym(2);
        pointKey3 = tagSym(3);
        pointKey4 = tagSym(4);
        
        graph.add(GenericProjectionFactorCal3_S2(measured1, projectionNoise, ...
                        poseKey, pointKey1, k));
        graph.add(GenericProjectionFactorCal3_S2(measured2, projectionNoise, ...
                        poseKey, pointKey2, k));
        graph.add(GenericProjectionFactorCal3_S2(measured3, projectionNoise, ...
                        poseKey, pointKey3, k));
        graph.add(GenericProjectionFactorCal3_S2(measured4, projectionNoise, ...
                        poseKey, pointKey4, k));
                    
        % add between factors for corners of tags
        graph.add(BetweenFactorPoint3(pointKey1, pointKey2, ...
                            Point3(TagSize, 0, 0), distanceNoise));
        graph.add(BetweenFactorPoint3(pointKey2, pointKey3, ...
                            Point3(0, TagSize, 0), distanceNoise));
        graph.add(BetweenFactorPoint3(pointKey3, pointKey4, ...
                            Point3(-TagSize, 0, 0), distanceNoise));
        graph.add(BetweenFactorPoint3(pointKey4, pointKey1, ...
                            Point3(0, -TagSize, 0), distanceNoise));
        graph.add(BetweenFactorPoint3(pointKey1, pointKey3, ...
                            Point3(TagSize, TagSize, 0), distanceNoise));
        graph.add(BetweenFactorPoint3(pointKey2, pointKey4, ...
                            Point3(-TagSize, TagSize, 0), distanceNoise));
    end
    
    % add between factors for camera poses
    if i < nrFrames
        I = Rot3([1 0 0; 0 1 0; 0 0 1]);
        T = Point3(0.001, 0.001, 0.001);
        graph.add(BetweenFactorPose3(cameraSymbols(i), cameraSymbols(i+1), ...
                  Pose3(I, T), poseNoise));
    end
end
disp('Factors were inserted...');

% give initial values
initials = Values;
for i = 1:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    point = cameraPoses{i, 2};
    point = Point3(point(1), point(2), point(3));
    pose = Pose3(Rot3(cameraPoses{i, 1}), point);
    initials.insert(cameraSymbols(i), pose);
    
    for j = 1:nrTags
        tag = frame(j, :);
        tagSyms = tagSymbols(tag(1));
        
        if ~initials.exists(tagSyms(1))
            wc = worldCoordinates(tag(1));
            p1 = Point3(wc(1), wc(2), 0);
            p2 = Point3(wc(3), wc(4), 0);
            p3 = Point3(wc(5), wc(6), 0);
            p4 = Point3(wc(7), wc(8), 0);
            initials.insert(tagSyms(1), p1);
            initials.insert(tagSyms(2), p2);
            initials.insert(tagSyms(3), p3);
            initials.insert(tagSyms(4), p4);
        end
    end
end
disp('Initial values were inserted...');

optimizer = LevenbergMarquardtOptimizer(graph, initials);
results = optimizer.optimizeSafely();

AllPosesComputed = zeros(nrFrames, 7);
nrTotalTags = size(tagSymbols, 1);
LandMarksComputed = zeros(nrTotalTags, 9);
tagIds = keys(tagSymbols);
tagIds = sort(cell2mat(tagIds), 'ascend');

for i = 1:nrFrames
    pose = results.at(cameraSymbols(i));
    quat = RotToQuat(pose.rotation().matrix());
    pos = pose.translation();
    AllPosesComputed(i, :) = [pos.x() pos.y() pos.z() ...
                              quat(1) quat(2) quat(3) quat(4)];
end

for i = 1:nrTotalTags
    tagId = tagIds(i);
    symbols = tagSymbols(tagId);
    p1 = results.at(symbols(1));
    p2 = results.at(symbols(2));
    p3 = results.at(symbols(3));
    p4 = results.at(symbols(4));
    LandMarksComputed(i, :) = [tagId p1.x() p1.y() p2.x() p2.y() p3.x() p3.y() ...
                               p4.x() p4.y()];
end
end
