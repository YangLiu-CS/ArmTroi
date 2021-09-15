function [threshold] = thresholdGenerate(hiddenState,Delta,Psi,index,num_continuity,factor_continuity)
% ***************functionality***************:
% calculate the threshold for the distance used to slim the comming search space using motion continuity
% ***************input***************:
% hiddenState: the state space
% Delta: the HMM probability for each state at each state space
% Psi: the position of the optimal previous state for this current state
% index: the timestamp for the current state space
% num_continuity: used to cal the threshold of the distance for motion continuity (how many previous locs used to cal the threshold)
% factor_continuity: used to calculate the threshold for motion continuity speedup (factor_continuity * the history loc interval)
% ***************output***************:
% threshold: the distance used to slim the comming search space using motion continuity

[~,psi_k] = max(Delta{index-1}(1,:)); % find the state with the maximal probability in the formaer state space
I = zeros();  
I(index-1,1) = psi_k;
for t = index-1-1:-1:index-1-(num_continuity-1)
    I(t,1) = Psi{t+1}(1,I(t+1,1)); % path backtracking to get the optimal path
end

% obtain the elbow's location sequence based on the optimal path
locElbow = zeros(3,num_continuity);
j = 0;
for t = index-1-(num_continuity-1):index-1
    j = j+1;
    locElbow(:,j) = hiddenState{t}(:,I(t,1));
end

% calculate the threshold using the mean of the distances between each two connected states
dis = zeros(1,num_continuity-1);
for i = 1:num_continuity-1
    dis(i) = disCalculate(locElbow(:,i),locElbow(:,i+1));
end
threshold = factor_continuity*mean(dis);
end