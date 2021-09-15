function [coordinateElbowFinal,coordinateWristFinal] = groundTruthFinal(kinectData,order,framelen,R,origin,la2,start_index,end_index,fIndex,lIndex,numW)
% ***************functionality***************:
% calculate the ground truth (elbow && wrist) from kinect: the final tracking location
% ***************input***************:
% kinectData: collected data from Kinect
% order: polynomial order of the Savitzky-Golay filter
% framelen: frame length of the Savitzky-Golay filter
% R: the rotation matrix from the kinect camera system to the torso system in ArmTroi
% origin: the estimated/measured origin coordinate of the sphere in the torso coordinate system (the coordinate of shoulder)
% la2: the radius of the sphere
% start_index: the time index of the kinect data at the beginning of the activity
% end_index: the time index of the kinect data at the ending of the activity
% fIndex: the start index (for the static part to cal the origin)
% lIndex: the end index (for the static part to cal the origin)
% numW: the number of tracked trace (used for alignment with kinect)
% ***************output***************:
% coordinateElbowFinal: the ground truth of elbow for the final tracking location
% coordinateWristFinal: the ground truth of wrist for the final tracking location

% the ground truth of elbow for the final tracking location
[kinectElbowBody,kinectWristBody] = kinectCalibration(kinectData,order,framelen,R,fIndex,lIndex);
[kinectElbowBodySphere] = kinectProjection(kinectElbowBody,origin,la2);

% this part is used to downsample the kinect data according to the smart watch data due to the different sample rate
downsampleFactor = floor((end_index-start_index)/numW);
if downsampleFactor < 1
    downsampleFactor = 1;
end
[kinectElbowBodySphereSparse] = kinectSparse(kinectElbowBodySphere(start_index:end_index,:)',downsampleFactor);
[kinectWristBodySparse] = kinectSparse(kinectWristBody(start_index:end_index,:)',downsampleFactor);

numK = size(kinectElbowBodySphereSparse,2);
kinectElbowBodySphereSparseFinal = zeros(3,numW);
kinectWristBodySparseFinal = zeros(3,numW);
for i = 1:3
    kinectElbowBodySphereSparseFinal(i,:) = resample(kinectElbowBodySphereSparse(i,:),numW,numK);
    kinectWristBodySparseFinal(i,:) = resample(kinectWristBodySparse(i,:),numW,numK);
end

coordinateElbowFinal = kinectElbowBodySphereSparseFinal;
coordinateWristFinal = kinectWristBodySparseFinal;

end