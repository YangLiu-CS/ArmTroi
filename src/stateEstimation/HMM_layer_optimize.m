function [hiddenState_s,size_state_s] = HMM_layer_optimize(window_size,index,Psi_f,Delta_f,I_f,hiddenState_f,locElbow_f,key_f,pointCloudSecond,hiddenState_s,size_state_s)
% ***************functionality***************:
% calculate the locElbow from the 1st-layer HMM result and utilize them as the key to get the search space for the 2nd-layer search
% ***************input***************:
% window_size: the size of samples that how many 1st-layer times steps are considered together to construct the 2nd-layer search space
% index: the last time step in the report window
% Psi_f: the initilized position of the optimal previous state for this current state (1st layer)
% Delta_f: the HMM probability for each state at each state space (1st layer)
% I_f: the optimal path from the HMM (1st layer)
% hiddenState_f: the state space (1st layer)
% locElbow_f: the 1st-layer HMM result (elbow location)
% key_f: the euler angle of the wrist to find the according point cloud (1st layer)
% pointCloudSecond: point clouds for the second-layer search
% hiddenState_s: the initilized state space (2nd layer)
% size_state_s: the initilized size of each state space (2nd layer)
% ***************output***************:
% hiddenState_s: the updated hiddenState_s
% size_state_s: the updated size_state_s

[~,psi_f] = max(Delta_f{index}(1,:));
I_f(index,1) = psi_f;
for t = index-1:-1:index-window_size+1
    I_f(t,1) = Psi_f{t+1}(1,I_f(t+1,1)); % path backtracking to get the optimal path
end

for t = index-window_size+1:index
    locElbow_f(:,t) = hiddenState_f{t}(:,I_f(t,1));
end
for i = index-window_size+1:index
    map_s = pointCloudSecond(matrix2str(key_f(:,i))); % in tracking process, a series of wrist's orientations; return type: map; each returned map has center (key) and the points in the range corresponding to the center (keyValue)
    key_s = matrix2str(locElbow_f(:,i)); % find the chosen center (key)
    hiddenState_s{i} = map_s(key_s); % the points in the range corresponding to the center (keyValue)
    size_state_s(i) = size(hiddenState_s{i}, 2);
end

end