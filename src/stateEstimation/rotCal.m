function [rotWatch,rotWrist,rotWristEuler] = rotCal(gameRotWatch,gameRotWatchNature,factor_times,factor)
% ***************functionality***************:
% calculate the rotation matrixes between several coordinate systems
% ***************input***************:
% gameRotWatch: the sensor data from gamerotationvector
% gameRotWatchNature: the original watch's orientation in world coordinate system (the first several samples of the sensor data from gamerotationvector)
% factor_times: tolZ = factor_times*factor; tolX = factor_times*factor; tolY = factor_times*factor
% factor: the granularity to divide the 5 DoFs for point cloud generation
% ***************output***************:
% rotWatch: rotation matrix from the watch coordinate system to the torso coordinate system
% rotWrist: rotation matrix from the wrist coordinate system to the torso coordinate system
% rotWristEuler: transform the rotWrist to the euler angle to find the according point cloud

[rotWatchWorld] = calculateRotWatchWorld(gameRotWatch); % rotation matrix from the watch coordinate system to the world coordinate system
[faceDirection] = faceDirectionCal(gameRotWatchNature); % rotation matrix from the world coordinate system to the torso coordinate system
[rotWristToWatch] = wristToWatch; % rotation matrix from the wrist coordinate system to the watch coordinate system

N = size(rotWatchWorld,3);
rotWrist = zeros(3,3,N);
rotWatch = zeros(3,3,N);
rotWristEuler = zeros(3,N); % euler angle

% calculate the rotWrist and transform it to the euler angle to find the according point cloud
tolZ = factor_times*factor;
tolX = factor_times*factor;
tolY = factor_times*factor;
for i = 1:N
    rotWrist(:,:,i) = faceDirection * rotWatchWorld(:,:,i) * rotWristToWatch; % rotation matrix from the wrist coordinate system to the torso coordinate system
    rotWatch(:,:,i) = faceDirection * rotWatchWorld(:,:,i); % rotation matrix from the watch coordinate system to the torso coordinate system

    thetaZ = atan2(-rotWrist(1,2,i),rotWrist(2,2,i));
    thetaX = asin(rotWrist(3,2,i));
    thetaY = atan2(-rotWrist(3,1,i),rotWrist(3,3,i));
    thetaZ = round(thetaZ/tolZ)*tolZ;
    thetaX = round(thetaX/tolX)*tolX;
    thetaY = round(thetaY/tolY)*tolY;
    
    rotWristEuler(:,i) = [thetaX;thetaY;thetaZ];
end

end