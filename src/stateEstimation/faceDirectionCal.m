function [faceDirection] = faceDirectionCal(gameRotWatchNature)
% ***************functionality***************:
% calculate the rotation matrix from the world coordinate system to the torso coordinate system
% ***************input***************:
% gameRotWatchNature: the original watch's orientation in world coordinate system (the first several samples of the sensor data from gamerotationvector)
% ***************output***************:
% faceDirection: rotation matrix from the world coordinate system to the torso coordinate system

th1 = 9*pi/6; % 270
th2 = pi/2; % 90
A1 = [1, 0, 0; 0, cos(th1), -sin(th1); 0, sin(th1), cos(th1)]; % rotate 270 along x axis
A2 = [cos(th2), -sin(th2), 0; sin(th2), cos(th2), 0; 0, 0, 1]; % rotate 90 along z axis
[rotWatchWorld] = calculateRotWatchWorld(gameRotWatchNature); % the rotation matrix from the nature watch coordinate system to the world coordinate system
[rotWatchTorso] = A1 * A2; % the rotation matrix from the nature watch coordinate system to the torso coordinate system
faceDirection = rotWatchTorso * rotWatchWorld'; % the rotation matrix from the world coordinate system to the torso coordinate system

end