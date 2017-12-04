function [poses, velocities] = extended_kalman_filter(AllPosesComputed, vel, ...
                                    acc_cam, gyro_cam, TLeftImgs, framePerIMU)
    warning off;
    % state symbols
    x1 = sym('x1', [3, 1]); % position
    x2 = sym('x2', [3, 1]); % euler
    x3 = sym('x3', [3, 1]); % velocity
    x4 = sym('x4', [3, 1]); % gyro bias
    x5 = sym('x5', [3, 1]); % acc bias
    % input symbols
    u1 = sym('u1', [3, 1]); % gyro measurement
    u2 = sym('u2', [3, 1]); % acc measurement
    % process noise symbols
    n1 = sym('n1', [3, 1]); 
    n2 = sym('n2', [3, 1]);
    n3 = sym('n3', [3, 1]);
    n4 = sym('n4', [3, 1]);
    n5 = sym('n5', [3, 1]);
    % measurement noise symbols
    v1 = sym('v1', [3, 1]);
    v2 = sym('v2', [3, 1]);
    v3 = sym('v3', [3, 1]);
    
    G = calc_G(x2(2), x2(3));
    R = calc_R(x2(1), x2(2), x2(3));
    
    % f function
    f1 = x3;
    f2 = G \ (u1 - x4 - n4);
    f3 = [0; 0; 9.81] + R * (u2 - x5 - n5);
    f4 = 0.00001*[1; 1; 1]; % TODO tune (gyro drift)
    f5 = 0.00001*[1; 1; 1]; % TODO tune (acc drift)
    jacobian_f_x = jacobian([f1; f2; f3; f4; f5], [x1; x2; x3; x4; x5]);
    jacobian_f_x = simplify(jacobian_f_x);
    jacobian_f_x = matlabFunction(jacobian_f_x);
    jacobian_f_n = jacobian([f1; f2; f3; f4; f5], [n1; n2; n3; n4; n5]);
    jacobian_f_n = simplify(jacobian_f_n);
    jacobian_f_n = matlabFunction(jacobian_f_n);
    f1 = matlabFunction(simplify(f1));
    f2 = matlabFunction(simplify(f2));
    f3 = matlabFunction(simplify(f3));
    
    % g function
    C = [eye(3), zeros(3, 3), zeros(3, 3), zeros(3, 3), zeros(3, 3);
         zeros(3, 3), eye(3), zeros(3, 3), zeros(3, 3), zeros(3, 3);
         zeros(3, 3), zeros(3, 3), eye(3), zeros(3, 3), zeros(3, 3)];
    x = [x1; x2; x3; x4; x5];
    v = [v1; v2; v3];
    g = C*x + v;
    jacobian_g_x = jacobian(g, [x1; x2; x3]);
    jacobian_g_x = simplify(jacobian_g_x);
    jacobian_g_x = matlabFunction(jacobian_g_x);
    jacobian_g_v = jacobian(g, [v1; v2; v3]);
    jacobian_g_v = simplify(jacobian_g_v);
    jacobian_g_v = matlabFunction(jacobian_g_v);
    g = matlabFunction(simplify(g));
    
    Qt = 0.02 * eye(15, 15); % process noise (tuned)
    Rt = 0.002 * eye(9, 9); % measurement noise (tuned)
    
    vt = zeros(9, 1);
    for i = 1:9
        vt(i) = Rt(i, i);
    end
    gyro_bias = [0.0018; 0.0018; 0.0018]; % bias of gyroscope (tuned)
    acc_bias = [0.1; 0.1; -0.05]; % bias of accelerometer (tuned)
    
    nrSteps = size(acc_cam, 1); % number of IMU steps
    nrFrames = size(AllPosesComputed, 1);
    % Get initial state (prior)
    initPose = AllPosesComputed(1, :);
    initVel = vel(1, :);
    initEuler = quat2eul([initPose(4), initPose(5), initPose(6), initPose(7)]);
    initEuler = [initEuler(1), initEuler(3), initEuler(2)]; % ZXY
    prevState = [initPose(1:3).'; initEuler.'; initVel.'; gyro_bias; acc_bias];
    mu_prev = prevState; % prior mean
    sigma_prev = Qt; % prior covariance
    poses = zeros(nrFrames, 6);
    velocities = zeros(nrFrames, 3);
    poses(1, :) = prevState(1:6).';
    velocities(1, :) = prevState(7:9).';
    
    for i = 2:nrSteps
        % get the frames associated to the IMU data
        prevFrame = framePerIMU(i-1);
        currFrame = framePerIMU(i);
        
        if currFrame > nrFrames
            break;
        end
        
        % compute timestamp difference
        t0 = TLeftImgs(prevFrame);
        t1 = TLeftImgs(currFrame);
        deltaT = t1 - t0;
        
        % get the measurement from iSAM2 and IMU
        pose = AllPosesComputed(currFrame, :);
        q = quat2eul([pose(4), pose(5), pose(6), pose(7)]);
        q = [q(1), q(3), q(2)];
        pdot = vel(currFrame, :);
        w_m = gyro_cam(i, :).';
        a_m = acc_cam(i, :).';
        
        % Compute jacobians
        At = jacobian_f_x(0.0, 0.0, 0.0, 0.0, 0.0, w_m(1), ...
                          w_m(3), a_m(1), a_m(2), a_m(3), mu_prev(4), ...
                          mu_prev(5), mu_prev(6), mu_prev(10), mu_prev(12), ...
                          mu_prev(13), mu_prev(14), mu_prev(15));
        Ut = jacobian_f_n(mu_prev(4), mu_prev(5), mu_prev(6));
        
        % Compute f with the most likely previous state
        f1_prev = f1(mu_prev(7), mu_prev(8), mu_prev(9));
        f2_prev = f2(0.0, 0.0, 0.0, w_m(1), w_m(2), w_m(3), mu_prev(4), ...
                     mu_prev(5), mu_prev(10), mu_prev(11), mu_prev(12));
        f3_prev = f3(0.0, 0.0, 0.0, a_m(1), a_m(2), a_m(3), mu_prev(4), ...
                     mu_prev(5), mu_prev(6), mu_prev(13), mu_prev(14), ...
                     mu_prev(15));
        f4_prev = f4;
        f5_prev = f5;
        f_prev = [f1_prev; f2_prev; f3_prev; f4_prev; f5_prev];
        
        % Predict next state
        Ft = eye(15) + At * deltaT;
        bt = (f_prev - At * mu_prev) * deltaT;
        Vt = Ut * deltaT;  
        mu_ = Ft * mu_prev + bt; % Predicted mean
        sigma_ = Ft * sigma_prev * Ft.' + Vt * Qt * Vt.'; % Predicted covariance
        
        % Update
        mu = mu_;
        sigma = sigma_;
        
        if prevFrame ~= currFrame % If frame changes, update
            % Compute jacobians
            Ct = jacobian_g_x();
            Wt = jacobian_g_v();
            % Compute predicted state's output
            g_ = g(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mu_(1), mu_(2), ...
               mu_(3), mu_(4), mu_(5), mu_(6), mu_(7), mu_(8), mu_(9));
            % Compute Kalman gain
            Kt = sigma_(1:9, 1:9)*Ct.'/(Ct*sigma_(1:9, 1:9)*Ct.'+Wt*Rt*Wt.');
            z = [pose(1:3).'; q.'; pdot.'] + vt; % add noise to iSAM2 data
            % Update mean and covariance
            mu(1:9) = mu_(1:9) + Kt * (z - g_);
            sigma(1:9, 1:9) = sigma_(1:9, 1:9) + Kt * Ct * sigma_(1:9, 1:9);
            poses(currFrame, :) = mu(1:6).';
            velocities(currFrame, :) = mu(7:9).';
        end
        mu_prev = mu;
        sigma_prev = sigma;
    end
end