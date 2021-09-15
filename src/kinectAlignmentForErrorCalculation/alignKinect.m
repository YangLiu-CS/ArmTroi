function [dataKinectAlignment,R_res,T_res] = alignKinect(dataKinect,dataRecovery)
% ***************functionality***************:
% make the ground truth and the recovered trace from ArmTroi aligned (time && translation && rotation)
% ***************input***************:
% dataKinect: the ground truth from kinect
% dataRecovery: the recovered trace from ArmTroi
% ***************output***************:
% dataKinectAlignment: the ground truth after alignment (time && translation && rotation)
% R_res: rotation alignment (used for error calculation of the wrist)
% T_res: translation alignment (used for error calculation of the wrist)

% doing time alignment on dataKinect with dataRecovery
dataKinectTime = dataKinect(:,1:size(dataRecovery,2));

% doing translation and rotation alignment on dataKinect with dataRecovery
[R_res,T_res] = umeyama(dataKinectTime,dataRecovery);
dataKinectAlignment = R_res*dataKinectTime+T_res;

end