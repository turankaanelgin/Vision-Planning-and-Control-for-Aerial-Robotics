%% Load data
addpath('data');
trajName = 'Mountain'; % Change this to desired trajectory
save = false; % Do not change this (in order to prevent saving figures)
load('CalibParams.mat');
load(sprintf('Data%s', trajName));

%% Handle IMU data
filteredTimestamps = align_timestamps(TLeftImgs, IMU);
[acc_cam, gyro_cam] = IMU2Camera(filteredTimestamps, IMU, qIMUToC, TIMUToC);

%% Generate poses and velocities
[AllPosesComputed, GTSAMPoses, vel, GTSAMVel, TLeftImgs] = ...
                        remove_invalid_data(PoseiSAM2, VeliSAM2, PoseGTSAM, ...
                                                        VelGTSAM, TLeftImgs);
framePerIMU = compute_closest_frames(IMU, filteredTimestamps, TLeftImgs);

%% Kalman filter
clc
close all
[poses, velocities] = extended_kalman_filter(AllPosesComputed, vel, ...
                                 acc_cam, gyro_cam, TLeftImgs, framePerIMU);
[poses, AllPosesComputed, GTSAMPoses] = remove_unaligned_poses(poses, ...
                                            AllPosesComputed, GTSAMPoses);
[velocities, VeliSAM2, VelGTSAM] = remove_unaligned_velocities(velocities, ...
                                                        VeliSAM2, VelGTSAM);
poses = adjust_poses(poses);
velocities = adjust_velocities(velocities);

filterGTSAMPoses = zeros(size(GTSAMPoses));
index = 1;
for i = 1:size(GTSAMPoses, 1)
    if any(GTSAMPoses(i, :))
        filterGTSAMPoses(index, :) = GTSAMPoses(i, :);
        index = index + 1;
    end
end
GTSAMPoses = filterGTSAMPoses(1:index-1, :);

%% Plot Positions

% x-axis
figure;
plot(poses(:, 1));
hold on
plot(AllPosesComputed(:, 1));
plot(GTSAMPoses(:, 1));
legend('EKF', 'iSAM2', 'GTSAM');
title('Positions (x-axis)');
xlabel('Frame number');
ylabel('Position (m) (x-axis)');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sPosX', ...
                        trajName, trajName), '.png']);
end

% y-axis
figure;
plot(poses(:, 2));
hold on
plot(AllPosesComputed(:, 2));
plot(GTSAMPoses(:, 2));
legend('EKF', 'iSAM2', 'GTSAM');
title('Positions (y-axis)');
xlabel('Frame number');
ylabel('Position (m) (y-axis)');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sPosY', ...
                        trajName, trajName), '.png']);
end

% z-axis
figure;
plot(poses(:, 3));
hold on
plot(AllPosesComputed(:, 3));
plot(GTSAMPoses(:, 3));
legend('EKF', 'iSAM2', 'GTSAM');
title('Positions (z-axis)');
xlabel('Frame number');
ylabel('Position (m) (z-axis)');
if strcmp(trajName, 'Mapping')
    ylim([0.5 1.5]);
end
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sPosZ', ...
                        trajName, trajName), '.png']);
end

%% Plot Velocities

% x-axis
figure;
plot(velocities(:, 1));
hold on
plot(VeliSAM2(:, 1));
plot(VelGTSAM(:, 1));
legend('EKF', 'iSAM2', 'GTSAM');
title('Velocities (x-axis)');
xlabel('Frame number');
ylabel('Velocity (m/s) (x-axis)');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sVelX', ...
                        trajName, trajName), '.png']);
end

% y-axis
figure;
plot(velocities(:, 2));
hold on
plot(VeliSAM2(:, 2));
plot(VelGTSAM(:, 2));
legend('EKF', 'iSAM2', 'GTSAM');
title('Velocities (y-axis)');
xlabel('Frame number');
ylabel('Velocity (m/s) (y-axis)');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sVelY', ...
                        trajName, trajName), '.png']);
end

% z-axis
figure;
plot(velocities(:, 3));
hold on
plot(VeliSAM2(:, 3));
plot(VelGTSAM(:, 3));
legend('EKF', 'iSAM2', 'GTSAM');
title('Velocities (z-axis)');
xlabel('Frame number');
ylabel('Velocity (m/s) (z-axis)');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sVelZ', ...
                        trajName, trajName), '.png']);
end

%% Plot Orientation

% x-axis
figure;
plot(poses(:, 4));
hold on
plot(AllPosesComputed(:, 4));
plot(GTSAMPoses(:, 4));
legend('EKF', 'iSAM2', 'GTSAM');
title('Orientations (roll)');
xlabel('Frame number');
ylabel('Roll');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sRoll', ...
                        trajName, trajName), '.png']);
end

% y-axis
figure;
plot(poses(:, 5));
hold on
plot(AllPosesComputed(:, 5));
plot(GTSAMPoses(:, 5));
legend('EKF', 'iSAM2', 'GTSAM');
title('Orientations (pitch)');
xlabel('Frame number');
ylabel('Pitch');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sPitch', ...
                        trajName, trajName), '.png']);
end

% z-axis
figure;
plot(poses(:, 6));
hold on
plot(AllPosesComputed(:, 6));
plot(GTSAMPoses(:, 6));
legend('EKF', 'iSAM2', 'GTSAM');
title('Orientations (yaw)');
xlabel('Frame number');
ylabel('Yaw');
if save
    img = getframe(gcf);
    imwrite(img.cdata, [sprintf('../Report/Images/%s/%sYaw', ...
                        trajName, trajName), '.png']);
end