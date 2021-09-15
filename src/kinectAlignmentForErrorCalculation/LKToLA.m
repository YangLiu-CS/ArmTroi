function [rotLKToLA] = LKToLA
% ***************functionality***************:
% calculate the rotation matrix from the torso coordinate system in Kinect to the torso coordinate system in ArmTroi
% ***************output***************:
% rotLKToLA: the rotation matrix from the torso coordinate system in Kinect to the torso coordinate system in ArmTroi

th1 = pi / 2; % 90
th2 = pi / 2; % 90
A1 = [cos(th1), -sin(th1), 0; sin(th1), cos(th1), 0; 0, 0, 1]; % rotate 90 along z axis
A2 = [1, 0, 0; 0, cos(th2), -sin(th2); 0, sin(th2), cos(th2)]; % rotate 90 along x axis
rotLKToLA = A1 * A2;

end