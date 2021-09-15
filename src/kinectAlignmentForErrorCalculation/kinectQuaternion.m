function [kinectLumbar,kinectWristLeft,kinectWristRight] = kinectQuaternion(kinectDataOrientation)
% ***************functionality***************:
% obtain the quaternion of lumbar, left wrist and right wrist from each own system to the camera system
% ***************input***************:
% kinectDataOrientation: collected data from Kinect
% ***************output***************:
% kinectLumbar: the quaternion sequence of torso from the defined lumbar system to the camera system [t, 4(x,y,z,w)] (defined: defined from Kinect SDK)
% kinectWristLeft: the quaternion sequence of left wrist from the defined left wrist system to the camera system
% kinectWristRight: the quaternion sequence of right wrist from the defined right wrist system to the camera system

% seperate the elbow and wrist data from the original collected Kinect data
num = size(kinectDataOrientation,1);
kinectLumbar = zeros(num/4,4);
kinectWristRight = zeros(num/4,4);
kinectWristLeft = zeros(num/4,4);
j = 1;
for i = 1:4:num-3
    kinectLumbar(j,:) = [kinectDataOrientation(i+1,1),kinectDataOrientation(i+1,2),kinectDataOrientation(i+1,3),kinectDataOrientation(i+1,4)];
    kinectWristRight(j,:) = [kinectDataOrientation(i+2,1),kinectDataOrientation(i+2,2),kinectDataOrientation(i+2,3),kinectDataOrientation(i+2,4)];
    kinectWristLeft(j,:) = [kinectDataOrientation(i+3,1),kinectDataOrientation(i+3,2),kinectDataOrientation(i+3,3),kinectDataOrientation(i+3,4)];
    j = j+1;
end

end