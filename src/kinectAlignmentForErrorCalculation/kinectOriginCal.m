function [kinectOrigin] = kinectOriginCal(kinectData,order,framelen,fIndex,lIndex)
% ***************functionality***************:
% this function is used to calibrate the origin coordinate of the kinect data after Savitzky-Golay filter
% ***************input***************:
% kinectData: collected data from Kinect
% order: polynomial order of the Savitzky-Golay filter
% framelen: frame length of the Savitzky-Golay filter
% fIndex: the start index (for the static part to cal the origin)
% lIndex: the end index (for the static part to cal the origin)
% ***************output***************:
% kinectOrigin: the origin coordinate of the kinect data in the kinect coordinate system

% obtain the origin coordinate at each frame
num = size(kinectData,1);
origin = zeros(num/21,3);
j = 1;
for i = 9:21:num-12
    origin(j,:) = [kinectData(i-1,1),kinectData(i-1,2),kinectData(i-1,3)];
    j = j+1;
end

% using Savizky-Golay filter
originAfterFilter = zeros(size(origin,1),3);
for i = 1:3
    originAfterFilter(:,i) = sgolayfilt(origin(:,i),order,framelen);
end

% calculate the average originAfterFilter as the final origin coordinate of the kinect data
kinectOrigin = zeros(1,3);
for i = 1:3
    kinectOrigin(i) = mean(originAfterFilter(fIndex:lIndex,i));
end

end