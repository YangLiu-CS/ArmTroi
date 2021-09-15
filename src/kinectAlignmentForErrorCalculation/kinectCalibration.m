function [kinectElbowBody,kinectWristBody] = kinectCalibration(kinectData,order,framelen,R,fIndex,lIndex)
% ***************functionality***************:
% this function is used to calibrate the kinect data using Savitzky-Golay filter
% ***************input***************:
% kinectData: collected data from Kinect
% order: polynomial order of the Savitzky-Golay filter
% framelen: frame length of the Savitzky-Golay filter
% R: the rotation matrix from the kinect camera system to the torso system in ArmTroi
% fIndex: the start index (for the static part to cal the origin)
% lIndex: the end index (for the static part to cal the origin)
% ***************output***************:
% kinectElbowBody: location sequences of elbow in the torso coordinate system in ArmTroi
% kinectWristBody: location sequences of wrist in the torso coordinate system in ArmTroi

% seperate the elbow and wrist data from the original collected kinect data
num = size(kinectData,1);
kinectShoulder = zeros(num/21,3);
kinectElbow = zeros(num/21,3);
kinectWrist = zeros(num/21,3);
j = 1;
origin = kinectOriginCal(kinectData,order,framelen,fIndex,lIndex);  % the origin is obtained through average
for i = 9:21:num-12
    % left hand
    kinectShoulder(j,:) = [kinectData(i-6,1),kinectData(i-6,2),kinectData(i-6,3)]-origin;
    kinectElbow(j,:) = [kinectData(i+1,1),kinectData(i+1,2),kinectData(i+1,3)]-origin;
    kinectWrist(j,:) = [kinectData(i+3,1),kinectData(i+3,2),kinectData(i+3,3)]-origin;
    j = j+1;
end
% using Savizky-Golay filter
kinectShoulderAfterFilter = zeros(size(kinectShoulder,1),3);
kinectElbowAfterFilter = zeros(size(kinectElbow,1),3);
kinectWristAfterFilter = zeros(size(kinectWrist,1),3);
for i = 1:3
    kinectShoulderAfterFilter(:,i) = sgolayfilt(kinectShoulder(:,i),order,framelen);
    kinectElbowAfterFilter(:,i) = sgolayfilt(kinectElbow(:,i),order,framelen);
    kinectWristAfterFilter(:,i) = sgolayfilt(kinectWrist(:,i),order,framelen);
end

% translate the calibrated kinect data from the kinect coordinate system to the torso coordinate system
kinectShoulderBody = zeros(num/21,3);
kinectElbowBody = zeros(num/21,3);
kinectWristBody = zeros(num/21,3);
j = 1;

for i = 1:size(kinectElbowAfterFilter,1)
    tmpShoulder = R*kinectShoulderAfterFilter(i,:)';
    tmpElbow = R*kinectElbowAfterFilter(i,:)';
    tmpWrist = R*kinectWristAfterFilter(i,:)';
    kinectShoulderBody(j,:) = tmpShoulder;
    kinectElbowBody(j,:) = tmpElbow;
    kinectWristBody(j,:) = tmpWrist;
    j = j+1;
end

end