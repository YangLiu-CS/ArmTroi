function [locElbow] = HMM_report_optimize(index,Psi_s,Delta_s,I_s,hiddenState_s,locElbow)
% ***************functionality***************:
% calculate the reported location of the locElbow from the 2nd-layer HMM search 
% ***************input***************:
% index: the last time step in the report window
% Psi_s: the initilized position of the optimal previous state for this current state (2nd layer)
% Delta_s: the HMM probability for each state at each state space (2nd layer)
% I_s: the optimal path from the HMM (2nd layer)
% hiddenState_s: the state space (2nd layer)
% locElbow: the 2nd-layer HMM result (elbow location)
% ***************output***************:
% locElbow: the updated locElbow

[~,psi_s] = max(Delta_s{index}(1,:));
I_s(index,1) = psi_s;
for t = index-1:-1:1
    I_s(t,1) = Psi_s{t+1}(1,I_s(t+1,1)); % path backtracking to get the optimal path
end
for t = 1:index
    locElbow(:,t) = hiddenState_s{t}(:,I_s(t,1));
end

end