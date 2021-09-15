function [rotWatchToWrist] = wristToWatch
% ***************functionality***************:
% calculate the rotation matrix from the wrist coordinate system to the watch coordinate system
% ***************output***************:
% rotWatchToWrist: rotation matrix from the wrist coordinate system to the watch coordinate system

th1 = pi/2; % 90
th2 = pi; % 180
A1 = [cos(th1), 0, sin(th1); 0, 1, 0; -sin(th1), 0, cos(th1)]; % rotate 90 along y axis
A2 = [1, 0, 0; 0, cos(th2), -sin(th2); 0, sin(th2), cos(th2)]; % rotate 180 along x axis
rotWatchToWrist = A1 * A2; % rotation matrix from the wrist coordinate system to the watch coordinate system

end