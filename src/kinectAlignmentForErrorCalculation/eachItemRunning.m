function [elbowKinectAlignmentFinal,errorElbowOverallFinal,wristKinectAlignmentFinal,errorWristOverallFinal] = eachItemRunning(locElbow,locWrist,dataKinect,order,framelen,dataKinectOrien,origin,la2,start_index_kinect,end_index_kinect,fIndex,lIndex,numW)
% ***************functionality***************:
% calculate the ground truth (elbow && wrist) from kinect: the final tracking location, and make the ground truth and the recovered trace from ArmTroi aligned (time && translation && rotation) to calculate errors
% ***************input***************:
% locElbow: the reported elbow locations to the user from ArmTroi
% locWrist: the reported wrist locations to the user from ArmTroi
% dataKinect: collected skeleton data from Kinect
% order: polynomial order of the Savitzky-Golay filter
% framelen: frame length of the Savitzky-Golay filter
% dataKinectOrien: collected orientation data from Kinect
% origin: the estimated/measured origin coordinate of the sphere in the torso coordinate system (the coordinate of shoulder)
% la2: the radius of the sphere
% start_index_kinect: the time index of the kinect data at the beginning of the activity
% end_index_kinect: the time index of the kinect data at the ending of the activity
% fIndex: the start index (for the static part to cal the origin)
% lIndex: the end index (for the static part to cal the origin)
% numW: the number of tracked trace (used for alignment with kinect)
% ***************output***************:
% elbowKinectAlignmentFinal: the ground truth of elbow location after alignment (time && translation && rotation)
% errorElbowOverallFinal: : the euler error list of elbow across time
% wristKinectAlignmentFinal: the ground truth of wrist location after alignment (time && translation && rotation)
% errorWristOverallFinal: : the euler error list of wrist across time

[R] = kinectToTrunk(dataKinectOrien,fIndex,lIndex);

% using a time window to search the best start and end points of the ground truth due to out of sync betweent the IMU system and the kinect system (time alignment)
% window size of the searched time window
left_window_size = 18;
right_window_size = 18;
if start_index_kinect-left_window_size <= 0
    left_window_size = start_index_kinect-1;
end
if end_index_kinect+left_window_size > size(dataKinect,1)/21
    right_window_size = size(dataKinect,1)/21 - end_index_kinect;
end

errorElbowPrintFinal = 100;
errorWristPrintFinal = 100;
for i = -left_window_size:right_window_size
    for j = -left_window_size:right_window_size
        [coordinateElbowFinal,coordinateWristFinal] = groundTruthFinal(dataKinect,order,framelen,R,origin,la2,start_index_kinect+i,end_index_kinect+j,fIndex,lIndex,numW);
        [elbowKinectAlignment,R_res,T_res] = alignKinect(coordinateElbowFinal,locElbow);
        % doing alignment on coordinateWristFinal with locWrist
        wristKinectAlignment = R_res*coordinateWristFinal(:,1:size(locWrist,2))+T_res;
        [errorElbowOverall] = errorCalculation(elbowKinectAlignment,locElbow);
        [errorWristOverall] = errorCalculation(wristKinectAlignment,locWrist);
        errorElbowPrint = mean(errorElbowOverall);
        errorWristPrint = mean(errorWristOverall);
        if(errorElbowPrint + errorWristPrint < errorElbowPrintFinal + errorWristPrintFinal)
            elbowKinectAlignmentFinal = elbowKinectAlignment;
            errorElbowPrintFinal = errorElbowPrint;
            errorElbowOverallFinal = errorElbowOverall;
            wristKinectAlignmentFinal = wristKinectAlignment;
            errorWristPrintFinal = errorWristPrint;
            errorWristOverallFinal = errorWristOverall;
        end
    end
end
end