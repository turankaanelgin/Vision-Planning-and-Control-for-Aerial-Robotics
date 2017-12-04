function AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, LandMarksComputed);
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

graph = NonlinearFactorGraph;
tagSymbols = containers.Map('KeyType', 'double', 'ValueType', 'any');
cameraSymbols = containers.Map('KeyType', 'double', 'ValueType', 'any');
% world coordinates of tags
worldCoordinates = containers.Map('KeyType', 'double', 'ValueType', 'any');
nrFrames = size(DetAll, 2);
cameraPoses = cell(nrFrames, 2);
nrLandmarks = size(LandMarksComputed, 1);

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
        DetAllReal{index} = DetAll{i};
        index = index + 1;
    end
end

nrFrames = nrValidFrames;
DetAll = DetAllReal;

% create symbols
for i = 1:nrFrames
    cameraSymbols(i) = symbol('x', i);
end

for i = 1:nrLandmarks
    landmark = LandMarksComputed(i, :);
    tagId = landmark(1);
    tagSymbols(tagId) = [symbol('a', tagId), symbol('b', tagId), ...
                         symbol('c', tagId), symbol('d', tagId)];
    worldCoordinates(landmark(1)) = landmark(2:9);
end

% estimate camera poses
initials = Values;
cameraParams = cameraParameters('IntrinsicMatrix', K.');
lastOrientation = [0 0 0; 0 0 0; 0 0 0];
lastLocation = [0 0 0];
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
        
        % give initial values
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
     
%     if isempty(imagePoints)
%         cameraPoses{i, 1} = [0 0 0; 0 0 0; 0 0 0];
%         cameraPoses{i, 2} = [0 0 0];
%         continue;
%     end
    % estimate camera pose
    try
        [orientation, location] = estimateWorldCameraPose(imagePoints, ...
                                    worldPoints, cameraParams, ...
                                    'MaxNumTrials', 500, 'Confidence', 99);
    catch
        cameraPoses{i, 1} = lastOrientation;
        cameraPoses{i, 2} = lastLocation;
        point = cameraPoses{i, 2};
        point = Point3(point(1), point(2), point(3));
        pose = Pose3(Rot3(cameraPoses{i, 1}), point);
        initials.insert(cameraSymbols(i), pose);
        continue;
    end
    cameraPoses{i, 1} = orientation;
    cameraPoses{i, 2} = location;
    lastOrientation = orientation;
    lastLocation = location;
    point = cameraPoses{i, 2};
    point = Point3(point(1), point(2), point(3));
    pose = Pose3(Rot3(cameraPoses{i, 1}), point);
    initials.insert(cameraSymbols(i), pose);
end
disp('Initial camera poses were estimated...')

% add priors and between factors
posePriorNoise  = noiseModel.Diagonal.Sigmas([0.1 0.1 0.1 0.3 0.3 0.3]');
priorPoint = cameraPoses{1, 2};
priorPoint = Point3(priorPoint(1), priorPoint(2), priorPoint(3));
posePriorMean = Pose3(Rot3(cameraPoses{1, 1}), priorPoint); 
graph.add(PriorFactorPose3(cameraSymbols(1), posePriorMean, posePriorNoise));

pointPriorNoise = noiseModel.Isotropic.Sigma(3, 0.01);
distanceNoise = noiseModel.Isotropic.Sigma(3, 0.5);
isam = ISAM2();
isam.update(graph, initials);
for i = 1:nrLandmarks
    landmark = LandMarksComputed(i, :);
    symbols = tagSymbols(landmark(1));
    p1 = Point3(landmark(2), landmark(3), 0);
    p2 = Point3(landmark(4), landmark(5), 0);
    p3 = Point3(landmark(6), landmark(7), 0);
    p4 = Point3(landmark(8), landmark(9), 0);
    graph.add(PriorFactorPoint3(symbols(1), p1, pointPriorNoise));
    graph.add(PriorFactorPoint3(symbols(2), p2, pointPriorNoise));
    graph.add(PriorFactorPoint3(symbols(3), p3, pointPriorNoise));
    graph.add(PriorFactorPoint3(symbols(4), p4, pointPriorNoise));
    isam.update();
end
disp('Priors and between factors were added...')

% add projection and pose factors
projectionNoise = noiseModel.Isotropic.Sigma(2, 0.5);
poseNoise = noiseModel.Diagonal.Sigmas([0.1 0.1 0.1 0.3 0.3 0.3]');
k = Cal3_S2(K(1,1), K(2,2), K(1,2), K(1,3), K(2,3));
for i = 1:nrFrames
    frame = DetAll{i};
    nrTags = size(frame, 1);
    poseKey = cameraSymbols(i);
    for j = 1:nrTags
        tag = frame(j, :);
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
    end
    
    if i < nrFrames
        I = Rot3([1 0 0; 0 1 0; 0 0 1]);
        T = Point3(0.001, 0.001, 0.001);
        graph.add(BetweenFactorPose3(cameraSymbols(i), cameraSymbols(i+1), ...
                  Pose3(I, T), poseNoise));
    end
    isam.update();
end
disp('Projection and pose factors were added...')

% add between factors for tag points
for i = 1:nrLandmarks
    landmark = LandMarksComputed(i, :);
    symbols = tagSymbols(landmark(1));
    graph.add(BetweenFactorPoint3(symbols(1), symbols(2), ...
              Point3(TagSize, 0, 0), distanceNoise));
    graph.add(BetweenFactorPoint3(symbols(2), symbols(3), ...
              Point3(0, TagSize, 0), distanceNoise));
    graph.add(BetweenFactorPoint3(symbols(3), symbols(4), ...
              Point3(-TagSize, 0, 0), distanceNoise));
    graph.add(BetweenFactorPoint3(symbols(4), symbols(1), ...
              Point3(0, -TagSize, 0), distanceNoise));
    graph.add(BetweenFactorPoint3(symbols(1), symbols(3), ...
              Point3(TagSize, TagSize, 0), distanceNoise));
    graph.add(BetweenFactorPoint3(symbols(2), symbols(4), ...
              Point3(-TagSize, TagSize, 0), distanceNoise));
    isam.update();
end

results = isam.calculateEstimate();

AllPosesComputed = zeros(nrFrames, 7);
for i = 1:nrFrames
    pose = results.at(cameraSymbols(i));
    quat = RotToQuat(pose.rotation().matrix());
    pos = pose.translation();
    AllPosesComputed(i, :) = [pos.x() pos.y() pos.z() ...
                              quat(1) quat(2) quat(3) quat(4)];
end
end
