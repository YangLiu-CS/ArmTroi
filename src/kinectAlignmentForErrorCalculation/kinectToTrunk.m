function [kinectToTrunk] = kinectToTrunk(kinectDataOrientation,fIndex,lIndex)
% ***************functionality***************:
% calculate the rotation matrix from the kinect camera system to the torso system in ArmTroi
% ***************input***************:
% kinectDataOrientation: collected orientation data from Kinect
% fIndex: the start index (for the static part to cal the origin)
% lIndex: the end index (for the static part to cal the origin)
% ***************output***************:
% kinectToTrunk: the rotation matrix from the kinect camera system to the torso system in ArmTroi

[rotMatrixLumbar,~,~] = kinectRotMatrix(kinectDataOrientation);

rotLToR = zeros(3,3);
rotLToR(1,1) = mean(rotMatrixLumbar(1,1,fIndex:lIndex));
rotLToR(1,2) = mean(rotMatrixLumbar(1,2,fIndex:lIndex));
rotLToR(1,3) = mean(rotMatrixLumbar(1,3,fIndex:lIndex));
rotLToR(2,1) = mean(rotMatrixLumbar(2,1,fIndex:lIndex));
rotLToR(2,2) = mean(rotMatrixLumbar(2,2,fIndex:lIndex));
rotLToR(2,3) = mean(rotMatrixLumbar(2,3,fIndex:lIndex));
rotLToR(3,1) = mean(rotMatrixLumbar(3,1,fIndex:lIndex));
rotLToR(3,2) = mean(rotMatrixLumbar(3,2,fIndex:lIndex));
rotLToR(3,3) = mean(rotMatrixLumbar(3,3,fIndex:lIndex));

[rotLKToLA] = LKToLA;
kinectToTrunk = rotLKToLA * rotLToR';

end