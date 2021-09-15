function [Psi,Delta,weight] = HMM_initial_optimize(O,hiddenState,size_state,Psi,Delta,varAccX,varAccY,varAccZ,time1,time2,top_num)
% ***************functionality***************:
% the optimal path selection based on the first three state spaces
% ***************input***************:
% O: observed acc: 3 x T-2, each row is the acc in x/y/z axis in the torso coordinate system
% hiddenState: the initilized state space
% size_state: the initilized size of each state space
% Psi: the initilized position of the optimal previous state for this current state
% Delta: the initilized HMM probability for each state at each state space
% varAccX/varAccY/varAccZ: the variance of the measured acc of the elbow (axis X, Y, Z)
% time1: acc time stamps
% time2: average time stampes between two samples of time1
% top_num: the top num transitions from t-2 --> t will be considered when t is coming
% ***************output***************:
% Psi: the updated Psi
% Delta: the updated Delta
% weight: the weight to indicate the importance of each state

b0 = 1 / (size_state(1) * size_state(2) * size_state(3));
Psi{2}  = zeros(1,size_state(2));
Psi{3}  = zeros(top_num,size_state(3));
Delta{3}  = zeros(top_num,size_state(3));
weight = zeros(1,size_state(3));
for k = 1 : size_state(3)
    Delta_2 = zeros(1,size_state(2));
    for j = 1 : size_state(2)
        for i = 1 : size_state(1)
            v1 = (hiddenState{2}(:,j) - hiddenState{1}(:,i) ) / time1(1);
            v2 = (hiddenState{3}(:,k)  - hiddenState{2}(:,j) ) / time1(2);
            a1  = (v2-v1)/time2(1);
            b1 = (1/(sqrt(2*pi) * varAccX)) * exp( -(a1(1) - O(1,1))^2 / (2*varAccX*varAccX) ); % probability of acc x
            b2 = (1/(sqrt(2*pi) * varAccY)) * exp( -(a1(2) - O(2,1))^2 / (2*varAccY*varAccY) ); % probability of acc y
            b3 = (1/(sqrt(2*pi) * varAccZ)) * exp( -(a1(3) - O(3,1))^2 / (2*varAccZ*varAccZ) ); % probability of acc z
            p = b0 * b1 * b2 * b3;
            if (p > Delta_2(j))
                Delta_2(j) = p;
                Psi{2}(j) = i; % put the Psi cell
            end
            currentWeight = b1 * b2 * b3;
            if currentWeight > weight(k)
                weight(k) = currentWeight;
            end
        end
    end
    [max_delta_2_all,psi_all] = sort(Delta_2,'descend'); % find the suboptimal probability
    if size_state(2) < top_num
        for index = 1:size_state(2)
            Psi{3}(index,k) = psi_all(index); % put the Psi cell
            Delta{3}(index,k) = max_delta_2_all(index); % put the Delta cell
        end
        for index = size_state(2)+1:top_num
            Psi{3}(index,k) = psi_all(1); % put the Psi cell
            Delta{3}(index,k) = max_delta_2_all(1); % put the Delta cell
        end
    else
        for index = 1:top_num
            Psi{3}(index,k) = psi_all(index); % put the Psi cell
            Delta{3}(index,k) = max_delta_2_all(index); % put the Delta cell
        end
    end
end

end