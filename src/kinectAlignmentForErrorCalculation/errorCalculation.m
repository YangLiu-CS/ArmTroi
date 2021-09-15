function [errorOverall] = errorCalculation(dataKinectAlignment,dataRecovery)
% ***************functionality***************:
% calculate errors between the aligned ground truth and the recovered trace
% ***************input***************:
% dataKinectAlignment: the ground truth after alignment (time && translation && rotation)
% dataRecovery: the recovered trace from ArmTroi
% ***************output***************:
% errorOverall: the euler error list across time

num = size(dataKinectAlignment,2);
errorAxisOverall = zeros(3,num);
errorOverall = zeros(1,num);
for i = 1:3
    errorAxisOverall(i,:) = abs(dataKinectAlignment(i,:) - dataRecovery(i,:));
end
for i = 1:num
    errorOverall(i) = sqrt(errorAxisOverall(1,i)^2+errorAxisOverall(2,i)^2+errorAxisOverall(3,i)^2);
end

end