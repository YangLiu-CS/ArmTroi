function [rotWristEuler,rotWrist,accElbow,time1,time2] = armTrack(gameRotWatchNature,gameRotWatch,accWatch,la3,factor_times,factor)
% ***************functionality***************:
% the calculation before HMM
% ***************input***************:
% gameRotWatchNature: the original watch's orientation in world coordinate system (the first several samples of the sensor data from gamerotationvector)
% gameRotWatch: the sensor data from gamerotationvector
% accWatch: the sensor data from acclerometer
% la3: the lower arm length
% factor_times: tolZ = factor_times*factor; tolX = factor_times*factor; tolY = factor_times*factor
% factor: the granularity to divide the 5 DoFs for point cloud generation
% ***************output***************:
% rotWristEuler: transform the rotWrist to the euler angle to find the according point cloud
% rotWrist: rotation matrix from the wrist coordinate system to the torso coordinate system
% accElbow: acc of the elbow in torso coordinate system from sensor measurement for HMM derivation
% time1: acc time stamps
% time2: average time stampes between two samples of time1

% downsample to 5Hz
[accWatch] = accSparse(accWatch);
[gameRotWatch] = rotSparse(gameRotWatch);
% calculate the rotation matrixes between several coordinate systems
[rotWatch,rotWrist,rotWristEuler] = rotCal(gameRotWatch,gameRotWatchNature,factor_times,factor);
% derive the acc of elbow in torso coordinate system from the sensor measurement
[accElbow,time1,time2] = accElbowFromWrist(rotWatch,rotWrist,accWatch,la3);

end