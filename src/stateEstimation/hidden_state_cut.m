function [hiddenStateCut] = hidden_state_cut(preLocation,hiddenState,threshold)
% ***************functionality***************:
% cut the original seach space using motion continuity
% ***************input***************:
% preLocation: the best choice (elbow location) from HMM for the previous frame (t-1)
% hiddenState: the original search space for the current frame (t): 3 x N
% threshold: the distance used to slim the comming search space using motion continuity
% ***************output***************:
% hiddenStateCut: the cutted seach space based on motion continuity for the current frame (t)

% traverse the original seach space at current time, discard the points which are far away from the preLocation more than a threshold
num = size(hiddenState,2);
hiddenStateCut = zeros(3,num);
j = 0;
for i = 1:num
	x_now = hiddenState(1,i);
	y_now = hiddenState(2,i);
	z_now = hiddenState(3,i);
	x_pre = preLocation(1);
	y_pre = preLocation(2);
	z_pre = preLocation(3);
	dis = sqrt((x_now-x_pre)*(x_now-x_pre)+(y_now-y_pre)*(y_now-y_pre)+(z_now-z_pre)*(z_now-z_pre));
	if dis <= threshold
		j=j+1;
		hiddenStateCut(:,j) = hiddenState(:,i);
	end
end
hiddenStateCut(:,j+1:num) = [];
% numC = size(hiddenStateCut,2)

end