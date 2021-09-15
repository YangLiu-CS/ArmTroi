function [locElbow,locElbowOptimal,locElbowPredictP,locElbowPredictH,locWrist,locWristOptimal,realtimeFactor,numW] = ArmTroiSearch_realtime_optimize(gameRotWatchNature,gameRotWatch,accWatch,pointCloudFirst,pointCloudSecond,la3,latency,warmLatency,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num,factor_times,factor)
% ***************functionality***************:
% the basic workflow of arm tracking with report window (in real time) in ArmTroi
% ***************input***************:
% gameRotWatchNature: the original watch's orientation in world coordinate system (the first several samples of the sensor data from gamerotationvector)
% gameRotWatch: the sensor data from gamerotationvector
% accWatch: the sensor data from acclerometer
% pointCloudFirst: point clouds for the first-layer search
% pointCloudSecond: point clouds for the second-layer search
% la3: the lower arm length
% latency: the report window (e.g., one second)
% warmlatency: the interval (number of the report window) for non-realtime report during warming up
% boolContinuity: indicate whether using motion continuity or not
% num_continuity: used to cal the threshold of the distance for motion continuity (how many previous locs used to cal the threshold)
% factor_continuity: used to calculate the threshold for motion continuity speedup (factor_continuity * the history loc interval)
% boolWeight: indicate whether combining the weight acceleration for each former search space
% updateMethod: indicate using weight order method or particle filter method to reconstruct state space at t-1
% factor_weight: only top factor_weght*size_state(t-1) states at t-1 th frame are kept
% top_num: the top num transitions from t-2 --> t will be considered when t is coming
% factor_times: tolZ = factor_times*factor; tolX = factor_times*factor; tolY = factor_times*factor
% factor: the granularity to divide the 5 DoFs for point cloud generation
% ***************output***************:
% locElbow: the reported elbow locations to the user from ArmTroi
% locElbowOptimal: the optimal path (using all observations from t=1:T (whole trace) to choose the optimal path)(just for analysing, since the 2nd-layer construction is still realtime)
% locElbowPredictP: the predicted previous location using Particle filter (for speeding up through motion continuity)
% locElbowPredictH: the predicted previous location using the HMM (maximal prob.) (for speeding up through motion continuity)
% locWrist: the reported wrist locations to the user from ArmTroi
% locWristOptimal: the optimal wrist locations from locElbowOptimal
% realtimeFactor: the cost time of tracking / the trace time
% numW: the number of tracked trace (used for alignment with kinect)

tic;

% the calculation before HMM
[rotWristEuler,rotWrist,accElbow,time1,time2] = armTrack(gameRotWatchNature,gameRotWatch,accWatch,la3,factor_times,factor);
numW = size(rotWristEuler,2);

% HMM tracks the elbow's locations using two-layers search (in real time: coming one-window sensory date, reporting one-window tracking result)
[locElbow,locElbowOptimal,locElbowPredictP,locElbowPredictH,locWrist,locWristOptimal] = HMM_realtime_optimize(rotWrist,rotWristEuler,accElbow,pointCloudFirst,pointCloudSecond,time1,time2,la3,latency,warmLatency,boolContinuity,num_continuity,factor_continuity,boolWeight,updateMethod,factor_weight,top_num);

elapsedTime = toc;
traceTime = size(locElbow,2)/5;
realtimeFactor = elapsedTime/traceTime;
% timePrint = [elapsedTime,traceTime,realtimeFactor]

end