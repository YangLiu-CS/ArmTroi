function [rotMatrixLumbar,rotMatrixWristLeft,rotMatrixWristRight] = kinectRotMatrix(kinectDataOrientation)
% ***************functionality***************:
% calculate the rotation matrixs of lumbar, left wrist and right wrist from each own system to the camera system
% ***************input***************:
% kinectDataOrientation: collected data from Kinect
% ***************output***************:
% rotMatrixLumbar: rotation matrix from the defined lumbar system to the camera system (defined: defined from Kinect SDK)
% rotMatrixWristLeft: rotation matrix from the defined left wrist system to the camera system
% rotMatrixWristRight: rotation matrix from the defined right wrist system to the camera system

[kinectLumbar,kinectWristLeft,kinectWristRight] = kinectQuaternion(kinectDataOrientation);
[rotMatrixLumbar] = rotMatrixCal(kinectLumbar');
[rotMatrixWristLeft] = rotMatrixCal(kinectWristLeft');
[rotMatrixWristRight] = rotMatrixCal(kinectWristRight');

end