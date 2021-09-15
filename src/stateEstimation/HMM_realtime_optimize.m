function [locElbow,locElbowOptimal,locElbowPredictP,locElbowPredictH,locWrist,locWristOptimal] = HMM_realtime_optimize(rotWrist,rotWristEuler,accElbow,pointCloudFirst,pointCloudSecond,time1,time2,la3,latency,warmLatency,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num)
% ***************functionality***************:
% HMM tracks the elbow's locations using two-layers search (in real time: coming one-window sensory date, reporting one-window tracking result)
% ***************input***************:
% rotWrist: rotation matrix from the wrist coordinate system to the torso coordinate system
% rotWristEuler: the euler angle of the wrist to find the according point cloud
% accElbow: acc of the elbow in the torso coordinate system from sensor measurement for HMM derivation
% pointCloudFirst: point clouds for the first-layer search
% pointCloudSecond: point clouds for the second-layer search
% time1: acc time stamps
% time2: average time stampes between two samples of time1
% la3: the lower arm length
% latency: the report window
% warmlatency: the interval (number of the report window) for non-realtime report during warming up
% boolContinuity: indicate whether using motion continuity or not
% num_continuity: used to cal the threshold of the distance for motion continuity (how many previous locs used to cal the threshold)
% factor_continuity: used to calculate the threshold for motion continuity speedup (factor_continuity * the history loc interval)
% boolWeight: indicate whether combining the weight acceleration for each former search space
% updateMethod: indicate using weight order method or particle filter method to reconstruct state space at t-1
% factor_weight: only top factor_weght*size_state(t-1) states at t-1 th frame are kept
% top_num: the top num transitions from t-2 --> t will be considered when t is coming
% ***************output***************:
% locElbow: reported elbow locations to the user
% locElbowOptimal: the optimal path (using all observations from t=1:T (whole trace) to choose the optimal path)(just for analysing, since the 2nd-layer construction is still realtime)
% locElbowPredictP: the predicted previous location using Particle filter (for speeding up through motion continuity)
% locElbowPredictH: the predicted previous location using the HMM (maximal prob.) (for speeding up through motion continuity)
% locWrist: the reported wrist locations to the user from locElbow
% locWristOptimal: the optimal wrist locations from locElbowOptimal

% observed acc: 3 x T-2, each row is the acc in x/y/z axis in the torso coordinate system
O = accElbow;

% create state space based on the point cloud
% for each wrist'orientation, find the point cloud, the type of each point cloud is map
% calculate the number of state for each point cloud
K = size(rotWristEuler,2);
key_f = zeros(3,K);
size_state_f = zeros(1,K);
hiddenState_f = cell(K,1);
for i = 1:K
    key_f(:,i) = rotWristEuler(:,i);
    size_state_f(i) = size(pointCloudFirst(matrix2str(key_f(:,i))),2);
    hiddenState_f{i} = pointCloudFirst(matrix2str(key_f(:,i)));
end

Psi_f = cell(K, 1);
Delta_f = cell(K, 1);
Psi_s = cell(K, 1);
Delta_s = cell(K, 1);

% initialize the priori probability
Psi_f{1}  = zeros(1,size_state_f(1));
Delta_f{1}  = ones(1,size_state_f(1)) * (1/size_state_f(1));
Delta_f{2}  = ones(1,size_state_f(2)) * (1/size_state_f(2));

% the standard variance of the observed accs, which is used to calculate the transition probability
varAccX = 5*std(O(1,:));
varAccY = 5*std(O(2,:));
varAccZ = 5*std(O(3,:));

% the optimal path selection based on the first three state spaces for the 1st-layer search
[Psi_f,Delta_f,weight_f] = HMM_initial_optimize(O,hiddenState_f,size_state_f,Psi_f,Delta_f,varAccX,varAccY,varAccZ,time1,time2,top_num);

% the optimal path selection based on the first three state spaces for the 2nd-layer search
I_f = [];
locElbow_f = zeros(3,K);
size_state_s = zeros(1,K);
hiddenState_s = cell(K,1);
window_size = 3;
[hiddenState_s,size_state_s] = HMM_layer_optimize(window_size,3,Psi_f,Delta_f,I_f,hiddenState_f,locElbow_f,key_f,pointCloudSecond,hiddenState_s,size_state_s);
Psi_s{1}  = zeros(1,size_state_s(1));
Delta_s{1}  = ones(1,size_state_s(1)) * (1/size_state_s(1));
Delta_s{2}  = ones(1,size_state_s(2)) * (1/size_state_s(2));
[Psi_s,Delta_s,weight_s] = HMM_initial_optimize(O,hiddenState_s,size_state_s,Psi_s,Delta_s,varAccX,varAccY,varAccZ,time1,time2,top_num);

locElbowPredictP = zeros(3,K); % predicted previous location using Particle filter
locElbowPredictH = zeros(3,K); % predicted previous location using the HMM (maximal prob.)
% calculate the predicted elbow location at time t for motion continuity speedup (t = 3)
% particle filter
reportHiddenState = zeros(3,size_state_s(3));
for i = 1:size_state_s(3)
    weight_s = weight_s./sum(weight_s);
    index = find(rand <= cumsum(weight_s),1);
    reportHiddenState(:,i) = hiddenState_s{3}(:,index); % higher weight, higher chance
end
% calculate the mean without outliers (X, percent, dim) (the mean excluding the highest and lowest k data values, where k = n*(percent/100)/2)
locElbowPredictP(:,3) = trimmean(reportHiddenState,20,2);
% HMM (maximal prob.)
[~,psi_s_3] = max(Delta_s{3}(1,:)); % find the state with the maximal probability in the formaer state space
locElbowPredictH(:,3) = hiddenState_s{3}(:,psi_s_3); % find the optimal location in the former state space

num_warm_latency = warmLatency*5; % the number of samples for warming up
num_latency = latency*5; % the number of sample for one report window

% calculate the values in the warmup stage (do not consider realtime, do not utilize motion continuity)
% default: warmLatency = 1 (1 second is a report window)
% using the predicted location from the 2nd-layer to speed up both 1st and 2nd layers through motion continuity
if K < num_warm_latency
    num_warm_latency = K;
end

% num_warm_latency = K; % without real-time feature

for t = 4:num_warm_latency
    [~,~,Psi_f,Delta_f,size_state_f,hiddenState_f,weight_f] = HMM_coming_optimize(0,t,O,size_state_f,hiddenState_f,Psi_f,Delta_f,varAccX,varAccY,varAccZ,locElbowPredictP,locElbowPredictH,weight_f,time1,time2,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num);
end
window_size = num_warm_latency-4+1;
[hiddenState_s,size_state_s] = HMM_layer_optimize(window_size,t,Psi_f,Delta_f,I_f,hiddenState_f,locElbow_f,key_f,pointCloudSecond,hiddenState_s,size_state_s);
for t = 4:num_warm_latency
    [locElbowPredictP,locElbowPredictH,Psi_s,Delta_s,size_state_s,hiddenState_s,weight_s] = HMM_coming_optimize(0,t,O,size_state_s,hiddenState_s,Psi_s,Delta_s,varAccX,varAccY,varAccZ,locElbowPredictP,locElbowPredictH,weight_s,time1,time2,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num);
end

locElbow = zeros(3,K); % reported elbow locations to the user
locElbowOptimal = zeros(3,K); % the optimal path (using all observations from t=1:T (whole trace) to choose the optimal path)
I_s = [];
% using all observations from t=1:num_warm_latency to choose the optimal path
[locElbow] = HMM_report_optimize(num_warm_latency,Psi_s,Delta_s,I_s,hiddenState_s,locElbow);

% calculate the remaining values in Delta step by step
if num_latency == 0
    for t = num_warm_latency+1:K
        [~,~,Psi_f,Delta_f,size_state_f,hiddenState_f,weight_f] = HMM_coming_optimize(1,t,O,size_state_f,hiddenState_f,Psi_f,Delta_f,varAccX,varAccY,varAccZ,locElbowPredictP,locElbowPredictH,weight_f,time1,time2,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num);
        window_size = 1;
        [hiddenState_s,size_state_s] = HMM_layer_optimize(window_size,t,Psi_f,Delta_f,I_f,hiddenState_f,locElbow_f,key_f,pointCloudSecond,hiddenState_s,size_state_s);
        [locElbowPredictP,locElbowPredictH,Psi_s,Delta_s,size_state_s,hiddenState_s,weight_s] = HMM_coming_optimize(1,t,O,size_state_s,hiddenState_s,Psi_s,Delta_s,varAccX,varAccY,varAccZ,locElbowPredictP,locElbowPredictH,weight_s,time1,time2,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num);
        % here, coming one sensory data sample, reporting one tracking result, thus locElow == locElbowPredictH
        locElbow(:,t) = locElbowPredictH(:,t);
    end
    [locElbowOptimal] = HMM_report_optimize(K,Psi_s,Delta_s,I_s,hiddenState_s);
else
    for t = num_warm_latency+1:num_latency:floor(K/num_latency)*num_latency
        for i = 0:num_latency-1
            [~,~,Psi_f,Delta_f,size_state_f,hiddenState_f,weight_f] = HMM_coming_optimize(i+1,t+i,O,size_state_f,hiddenState_f,Psi_f,Delta_f,varAccX,varAccY,varAccZ,locElbowPredictP,locElbowPredictH,weight_f,time1,time2,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num);
        end
        window_size = num_latency;
        [hiddenState_s,size_state_s] = HMM_layer_optimize(window_size,t+i,Psi_f,Delta_f,I_f,hiddenState_f,locElbow_f,key_f,pointCloudSecond,hiddenState_s,size_state_s);
        for i = 0:num_latency-1
            [locElbowPredictP,locElbowPredictH,Psi_s,Delta_s,size_state_s,hiddenState_s,weight_s] = HMM_coming_optimize(1,t+i,O,size_state_s,hiddenState_s,Psi_s,Delta_s,varAccX,varAccY,varAccZ,locElbowPredictP,locElbowPredictH,weight_s,time1,time2,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num);
        end
        [locElbowOptimal] = HMM_report_optimize(t+i,Psi_s,Delta_s,I_s,hiddenState_s);
        locElbow(:,t:t+num_latency-1) = locElbowOptimal(:,t:t+num_latency-1);
    end
    locElbow = locElbow(:,1:floor(K/num_latency)*num_latency);
end
size_loc = size(locElbow,2);
locWrist = zeros(3,size_loc); % reported wrist locations to the user from locElbow
locWristOptimal = zeros(3,size_loc); % the optimal wrist locations from locElbowOptimal
for t = 1:size_loc
    locWrist(:,t) = locElbow(:,t) + (rotWrist(:,:,t) * [0;0;-la3]);
    locWristOptimal(:,t) = locElbowOptimal(:,t) + (rotWrist(:,:,t) * [0;0;-la3]);
end

end