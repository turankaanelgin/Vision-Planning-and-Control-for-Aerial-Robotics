function [acc_cam, gyro_cam] = IMU2Camera(filteredTimestamps, IMU, ...
                                          qIMUToC, TIMUToC)
    % Converts IMU frame to camera frame
    nrIMUData = length(filteredTimestamps);
    RIMUToC = QuatToRot(qIMUToC);
    acc_IMU = IMU(:, 5:7);
    gyro_IMU = IMU(:, 8:10);
    % extrinsic matrix
    extrinsics = zeros(4, 4);
    extrinsics(1:3, 1:3) = RIMUToC;
    extrinsics(1:3, 4) = TIMUToC;
    extrinsics(4, :) = [0, 0, 0, 1];
    acc_cam = zeros(nrIMUData, 3);
    gyro_cam = zeros(nrIMUData, 3);

    for i = 1:nrIMUData
        timestamp = filteredTimestamps(i);
        acc = [acc_IMU(timestamp, :).'; 1];
        gyro = [gyro_IMU(timestamp, :).'; 1];
        acc_res = extrinsics * acc;
        gyro_res = extrinsics * gyro;
        acc_cam(i, :) = acc_res(1:3).';
        gyro_cam(i, :) = gyro_res(1:3).';
    end
end