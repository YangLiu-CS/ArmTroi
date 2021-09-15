function [groundtruth] = kinectSparse(kinectData,N)
% ***************functionality***************:
% downsampling the kinectData
% ***************input***************:
% kinectData: initial kinectData
% ***************output***************:
% groundtruth: kinectData after downsampling

num = size(kinectData,2);
groundtruth = zeros(3,ceil(num/N));
j = 1;
for i = 1:N:ceil(num/N)*N-(N-1)
    groundtruth(:,j) = kinectData(:,i);
    j = j+1;
end

end