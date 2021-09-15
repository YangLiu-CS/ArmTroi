function [accNew] = accSparse(acc)
% ***************functionality***************:
% downsampling the acc to 5Hz
% ***************input***************:
% acc: acc in 50Hz
% ***************output***************:
% accNew: acc in 5Hz

num = size(acc,1);
accNew = zeros(ceil(num/10),4);
j = 1;
for i = 1:10:ceil(num/10)*10-9
    accNew(j,:) = acc(i,:);
    j = j+1;
end
end