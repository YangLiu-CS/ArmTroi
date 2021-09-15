function [locElbowReportP,locElbowReportH,Psi,Delta,size_state,hiddenState,weight] = HMM_coming_optimize(t_continuity,t,O,size_state,hiddenState,Psi,Delta,varAccX,varAccY,varAccZ,locElbowReportP,locElbowReportH,weight,time1,time2,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num)
% ***************functionality***************:
% conducting the HMM search for each coming sample(s)
% ***************input***************:
% t_continuity: the time step adapting the motion continuity (the first time step in one report window)
% t: the current time step
% O: acc of the elbow in the torso coordinate system from sensor measurement for HMM derivation
% size_state: the size of each state space
% hiddenState: the state space
% Psi: the initilized position of the optimal previous state for this current state
% Delta: the initilized HMM probability for each state at each state space
% varAccX/varAccY/varAccZ: the variance of the measured acc of the elbow (axis X, Y, Z)
% locElbowReportP: the reported elbow location as the previous predicted result for motion continuity speedup (particle filter)
% locElbowReportH: the reported elbow location as the previous predicted result for motion continuity speedup (HMM maximal prob.)
% weight: the weight to indicate the importance of each state
% time1: acc time stamps
% time2: average time stampes between two samples of time1
% boolContinuity: indicate whether using motion continuity or not
% num_continuity: used to cal the threshold of the distance for motion continuity (how many previous locs used to cal the threshold)
% factor_continuity: used to calculate the threshold for motion continuity speedup (factor_continuity * the history loc interval)
% boolWeight: indicate whether combining the weight acceleration for each former search space
% updateMethod: indicate using weight order method or particle filter method to reconstruct state space at t-1
% factor_weight: only top factor_weght*size_state(t-1) states at t-1 th frame are kept
% top_num: the top num transitions from t-2 --> t will be considered when t is coming
% ***************output***************:
% locElbowReportP: the reported elbow location as the previous predicted result for motion continuity speedup (HMM maximal prob.)
% locElbowReportH: the reported elbow location as the previous predicted result for motion continuity speedup (particle filter)
% Psi: the updated Psi
% Delta: the updated Delta
% size_state: the updated size of each state space
% hiddenState: the updated state space
% weight: the updated weight to indicate the importance of each state

% calculate the remaining values in Delta step by step
Psi{t}  = zeros(top_num,size_state(t));
Delta{t}  = zeros(top_num,size_state(t));
% motion continuity
if boolContinuity == 1 && t_continuity == 1
    % calculate the threshold for the distance used to slim the comming search space using motion continuity
    if num_continuity > t-1
        num_continuity = t-1;
    end
    pre_location = locElbowReportP(:,t-1);
    [threshold] = thresholdGenerate_optimize(hiddenState,Delta,Psi,t,num_continuity,factor_continuity);
    keepStates = hidden_state_cut(pre_location,hiddenState{t},threshold);
    if size(keepStates,2) >= 1/3*size_state(t)
        hiddenState{t} = keepStates;
        size_state(t) = size(hiddenState{t},2);
        Psi{t}  = zeros(top_num,size_state(t));
        Delta{t}  = zeros(top_num,size_state(t));
    end
end
% weight acceleration
if boolWeight == 1 && size_state(t-1) >= 10
    lastWeight = weight;
    lastWeight = lastWeight./sum(lastWeight);
    combineFormerHiddenState = [hiddenState{t-1};Psi{t-1};Delta{t-1}];
    chosenCombineFormerHiddenState = zeros(top_num*2+3,size_state(t-1));
    chosenCombineFormerHiddenState1 = zeros(top_num*2+3,size_state(t-1));
    chosenCombineFormerHiddenState2 = zeros(top_num*2+3,size_state(t-1));
    if updateMethod == 0
        % update method 1:
        for i = 1:size_state(t-1)
            index = find(rand <= cumsum(lastWeight),1);
            chosenCombineFormerHiddenState(:,i) = combineFormerHiddenState(:,index); % higher weight, higher chance
        end
        chosenCombineFormerHiddenStateRepeated = chosenCombineFormerHiddenState;
        % IndexChosenCombineFormerHiddenState: the index in chosenCombineFormerHiddenState for each ele from chosenCombineFormerHiddenState (Repeated)
        [chosenCombineFormerHiddenState,~,IndexChosenCombineFormerHiddenState] = unique(chosenCombineFormerHiddenStateRepeated','rows','stable');
        % assign higher weight to the state appearing more times
        IndexChosenCombineFormerHiddenState = sort(IndexChosenCombineFormerHiddenState);
        ind=histc(IndexChosenCombineFormerHiddenState,1:size(chosenCombineFormerHiddenState,1));
        assignWeight = ind/size(chosenCombineFormerHiddenStateRepeated,2);
        chosenCombineFormerHiddenState = chosenCombineFormerHiddenState';
    elseif updateMethod == 1
        % update method 2:
        [~,lastWeightIndex] = sort(lastWeight,'descend');
        for i = 1:size_state(t-1)
            chosenCombineFormerHiddenState(:,i) = combineFormerHiddenState(:,lastWeightIndex(i)); % higher weight, higher chance
        end
        chosenCombineFormerHiddenState = chosenCombineFormerHiddenState(:,1:floor(factor_weight*size_state(t-1)));
    elseif updateMethod == 2
        % combining update method 1 and 2:
        % method 1:
        for i = 1:size_state(t-1)
            index = find(rand <= cumsum(lastWeight),1);
            chosenCombineFormerHiddenState1(:,i) = combineFormerHiddenState(:,index); % higher weight, higher chance
        end
        % method 2:
        [~,lastWeightIndex] = sort(lastWeight,'descend');
        for i = 1:size_state(t-1)
            chosenCombineFormerHiddenState2(:,i) = combineFormerHiddenState(:,lastWeightIndex(i)); % higher weight, higher chance
        end
        chosenCombineFormerHiddenState2 = chosenCombineFormerHiddenState2(:,1:floor(factor_weight*size_state(t-1)));
        % combine
        % IndexChosenCombineFormerHiddenState: the index in chosenCombineFormerHiddenState for each ele from chosenCombineFormerHiddenState (Repeated)
        chosenCombineFormerHiddenStateRepeated = [chosenCombineFormerHiddenState1,chosenCombineFormerHiddenState2];
        [chosenCombineFormerHiddenState,~,IndexChosenCombineFormerHiddenState] = unique(chosenCombineFormerHiddenStateRepeated','rows','stable');
        % assign higher weight to the state appearing more times
        IndexChosenCombineFormerHiddenState = sort(IndexChosenCombineFormerHiddenState);
        ind=histc(IndexChosenCombineFormerHiddenState,1:size(chosenCombineFormerHiddenState,1));
        assignWeight = ind/size(chosenCombineFormerHiddenStateRepeated,2);
        chosenCombineFormerHiddenState = chosenCombineFormerHiddenState';
    end
    
    hiddenState{t-1} = chosenCombineFormerHiddenState(1:3,:);
    Psi{t-1} = chosenCombineFormerHiddenState(4:4+top_num-1,:);
    Delta{t-1} = chosenCombineFormerHiddenState(4+top_num:4+top_num+top_num-1,:);
    size_state(t-1) = size(chosenCombineFormerHiddenState,2);
end
% j: the current state in current state space
% i: the saved the optimal probability of each state in the former state space using the viterbi algorithm
weight = zeros(1,size_state(t));
for j = 1:size_state(t)
    % the optimal path of the jth state in the tth frame
    Delta_i = zeros(1,size_state(t-1));
    for i = 1:size_state(t-1)
        for k = 1:top_num % top_num optimal acc(t-2)-->acc(t-1)
            % transition probability: calculate in transition, because the acc can be caculated from the former state to the current state
            % acc calculation
            vi = (hiddenState{t-1}(:,i) - hiddenState{t-2}(:,Psi{t-1}(k,i)))/time1(t-2);
            vj = (hiddenState{t}(:,j) - hiddenState{t-1}(:,i))/time1(t-1);
            accij = (vj - vi) / time2(t-2);
            b1 = (1/(sqrt(2*pi) * varAccX)) * exp( -(accij(1) - O(1,t-2))^2 / (2*varAccX*varAccX) ); % probability of acc x
            b2 = (1/(sqrt(2*pi) * varAccY)) * exp( -(accij(2) - O(2,t-2))^2 / (2*varAccY*varAccY) ); % probability of acc y
            b3 = (1/(sqrt(2*pi) * varAccZ)) * exp( -(accij(3) - O(3,t-2))^2 / (2*varAccZ*varAccZ) ); % probability of acc z
            p = Delta{t-1}(k,i) * b1 * b2 * b3;
            if boolWeight == 1 && size_state(t-1) >= 10 && (updateMethod == 0 || updateMethod == 2)
                p = p*assignWeight(i);
            end
            if (p > Delta_i(i))
                Delta_i(i) = p;
            end
            currentWeight = b1 * b2 * b3;
            if currentWeight > weight(j)
                weight(j) = currentWeight;
            end
        end
    end
    [max_delta_i_all,psi_all] = sort(Delta_i,'descend'); % find the suboptimal probability
    if size_state(t-1) < top_num
        for index = 1:size_state(t-1)
            Psi{t}(index,j) = psi_all(index); % put the Psi cell
            Delta{t}(index,j) = max_delta_i_all(index); % put the Delta cell
        end
        for index = size_state(t-1)+1:top_num
            Psi{t}(index,j) = psi_all(1); % put the Psi cell
            Delta{t}(index,j) = max_delta_i_all(1); % put the Delta cell
        end
    else
        for index = 1:top_num
            Psi{t}(index,j) = psi_all(index); % put the Psi cell
            Delta{t}(index,j) = max_delta_i_all(index); % put the Delta cell
        end
    end
end
% calculate the predicted elbow location at time t for motion continuity speedup
% particle filter
reportHiddenState = zeros(3,size_state(t));
for i = 1:size_state(t)
    weight = weight./sum(weight);
    index = find(rand <= cumsum(weight),1);
    reportHiddenState(:,i) = hiddenState{t}(:,index); % higher weight, higher chance
end
% calculate the mean without outliers (X, percent, dim) (the mean excluding the highest and lowest k data values, where k = n*(percent/100)/2)
locElbowReportP(:,t) = trimmean(reportHiddenState,20,2);
% HMM (maximal prob.)
[~,psi_k] = max(Delta{t}(1,:)); % find the state with the maximal probability in the formaer state space
locElbowReportH(:,t) = hiddenState{t}(:,psi_k); % find the optimal location in the former state space

end