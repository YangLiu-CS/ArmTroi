function [rotNew] = rotSparse(rot)
% ***************functionality***************:
% downsampling the gameRotWatch to 5Hz
% ***************input***************:
% rot: gameRotWatch in 50Hz
% ***************output***************:
% rotNew: gameRotWatch in 5Hz

num = size(rot,1);
rotNew = zeros(ceil(num/10),5);
j = 1;
for i = 1:10:ceil(num/10)*10-9
    rotNew(j,:) = rot(i,:);
    j = j+1;
end
end