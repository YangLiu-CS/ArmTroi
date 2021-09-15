function [accElbow,time1,time2] = accElbowFromWrist(rotWatch,rotWrist,accWatch,la3)
% ***************functionality***************:
% derive the acc of elbow in torso coordinate system from the sensor measurement
% ***************input***************:
% rotWatch: rotation matrix from the watch coordinate system to torso coordinate system
% rotWrist: rotation matrix from the wrist coordinate system to the torso coordinate system
% accWatch: the sensor data from acclerometer (in watch coordinate system)
% la3: the lower arm length
% ***************output***************:
% accElbow: acc of the elbow in torso coordinate system from sensor measurement for HMM derivation
% time1: acc time stamps
% time2: average time stampes between two samples of time1

num = size(accWatch,1);
accWrist = zeros(3,num); % wrist acc in torso coordinate system
locWristtoElbow = zeros(3,num);
for n = 1:num
    accWrist(:,n) = rotWatch(:,:,n)*accWatch(n,2:4)';
    locWristtoElbow(:,n) = rotWrist(:,:,n)*[0;0;la3];
end

% time stamps
time1 = diff(accWatch(:,1))*10^(-9); % acc time stamps
time2 = zeros(num-2,1); % average time stampes between two samples of time1
for n = 1:num-2
    time2(n) = (time1(n)+time1(n+1))/2;
end

% second derivation of locWristtoElbow
vWristtoElbow = zeros(3,num-1);
accWristtoElbow = zeros(3,num-2);
for n = 1:3
     vWristtoElbow(n,:)= diff(locWristtoElbow(n,:))./time1';
     accWristtoElbow(n,:)= diff(vWristtoElbow(n,:))./time2';
end

% acc of wristplot(accCombineElbow3)
accElbow = accWrist(:,2:num-1) + accWristtoElbow; % matrix dim: 3 x num-2 (3 means x,y,z)

end