function inlierIdx = ransac(points, flows, Z, nrSamples, nrIterations, ...
                            threshold, K)
%     u0 = K(1, 3);
%     v0 = K(2, 3);
%     fx = K(1, 1);
%     fy = K(2, 2);
%     nrPoints = size(points, 1);
%     allVelocities = cell(nrIterations, 1);
%     allF = cell(nrIterations, 1);
%     allFlowVec = cell(nrIterations, 1);
%     samples = cell(nrIterations, 3);
%     
%     for i = 1:nrIterations
%         sampleIdx = datasample(1:nrPoints, 3, 'Replace', false);
%         sample = points(sampleIdx, :);
%         samples{i} = sampleIdx;
%         flowsPerSample = flows(sampleIdx, :);
%         f = zeros(6, 6);
%         flowVec = zeros(6, 1);
%         
%         index = 1;
%         for k = 1:3
%             x = sample(k, 1) - u0;
%             y = sample(k, 2) - v0;
%             f1 = fx * [-1/Z, 0, x/Z, x*y, -(x^2+1), y];
%             f2 = fy * [0, -1/Z, y/Z, y^2+1, -x*y, -x];
%             f(index, :) = f1;
%             f(index+1, :) = f2;
%             flowVec(index:index+1) = flowsPerSample(k, :);
%             index = index + 2;
%         end
%         
%         velocities = f \ flowVec;
%         allVelocities{i} = velocities;
%         allF{i} = f;
%         allFlowVec{i} = flowVec;
%     end
%     
%     errors = zeros(nrIterations, 1);
%     for i = 1:nrIterations
%         velocities = allVelocities{i};
%         totalError = 0;
%         
%         for j = 1:nrIterations
%             if i == j
%                 continue;
%             end
%             
%             f = allF{i};
%             flowVec = allFlowVec{j};
%             estimatedFlowVec = f * velocities;
%             error = sum(abs(flowVec - estimatedFlowVec));
%             totalError = totalError + error;
%         end
%         
%         errors(i) = totalError;
%     end
%     %mean(errors)
%     inliers = find(errors < threshold);
%     nrInliers = length(inliers);
%     inlierIdx = zeros(nrInliers, 1);
%     index = 1;
%     for i = 1:nrInliers
%         inlier = samples{inliers(i)};
%         for k = 1:3
%             if ~ismember(inlierIdx, inlier(k))
%                 inlierIdx(index) = inlier(k);
%                 index = index + 1;
%             end
%         end
%     end
%     inlierIdx = inlierIdx(1:index-1, :);
    
    u0 = K(1, 3);
    v0 = K(2, 3);
    fx = K(1, 1);
    fy = K(2, 2);
    nrPoints = size(points, 1);
    allVelocities = zeros(6, nrSamples);
    allSamples = cell(nrSamples, 1);
    
    % produce samples and estimate velocities for each
    for i = 1:nrSamples
        sampleIdx = datasample(1:nrPoints, 3, 'Replace', false);
        sample = points(sampleIdx, :);
        flowsPerSample = flows(sampleIdx, :);
        f = zeros(6, 6);
        flowVec = zeros(6, 1);
        
        index = 1;
        for k = 1:3
            x = sample(k, 1) - u0;
            y = sample(k, 2) - v0;
            f1 = fx * [-1/Z, 0, x/Z, x*y, -(x^2+1), y];
            f2 = fy * [0, -1/Z, y/Z, y^2+1, -x*y, -x];
            f(index, :) = f1;
            f(index+1, :) = f2;
            flowVec(index:index+1) = flowsPerSample(k, :)';
            index = index + 2;
        end
        
        velocities = f \ flowVec;
        allVelocities(:, i) = velocities;
        allSamples{i} = sampleIdx;
    end
    
    % compute inliers from velocities by fitting a line
    bestNrInliers = 0;
    bestInliers = [];
    for i = 1:nrIterations
        idx = randperm(nrSamples, 2); % select 2 points for fitting a line
        sample = allVelocities(:, idx);
        p1 = sample(:, 1);
        p2 = sample(:, 2);
        distances = zeros(nrSamples, 1);
        
        for j = 1:nrSamples
            p3 = allVelocities(:, j);
            lambda = dot(p2-p1, p2-p3) / dot(p1-p2, p1-p2);
            distances(j) = norm(lambda*p1 + (1-lambda)*p2 - p3);
        end
        inliers = find(abs(distances) < threshold);
        nrInliers = length(inliers);
        
        if nrInliers > bestNrInliers
            bestNrInliers = nrInliers;
            bestInliers = inliers;
        end
    end
    
    inlierIdx = zeros(bestNrInliers, 1);
    index = 1;
    for i = 1:bestNrInliers
        inlier = allSamples{bestInliers(i)};
        for k = 1:3
            if ~ismember(inlierIdx, inlier(k))
                inlierIdx(index) = inlier(k);
                index = index + 1;
            end
        end
    end
    inlierIdx = inlierIdx(1:index-1, :);
end