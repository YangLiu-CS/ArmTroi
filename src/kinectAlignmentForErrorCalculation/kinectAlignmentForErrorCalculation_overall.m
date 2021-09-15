N = 14;
kinectElbow = cell(1,N);
kinectWrist = cell(1,N);
errorElbow = cell(1,N);
errorWrist = cell(1,N);

[kinectElbow{1},errorElbow{1},kinectWrist{1},errorWrist{1}] = eachItemRunning(trackElbow{1},trackWrist{1},kinectSkeletonData{1,1},order,framelen,kinectOrienData{1,1},origin,la2,41,226,230,250,numW(1));
[kinectElbow{2},errorElbow{2},kinectWrist{2},errorWrist{2}] = eachItemRunning(trackElbow{2},trackWrist{2},kinectSkeletonData{1,2},order,framelen,kinectOrienData{1,2},origin,la2,35,220,220,240,numW(2));
[kinectElbow{3},errorElbow{3},kinectWrist{3},errorWrist{3}] = eachItemRunning(trackElbow{3},trackWrist{3},kinectSkeletonData{1,3},order,framelen,kinectOrienData{1,3},origin,la2,47,233,25,45,numW(3));
[kinectElbow{4},errorElbow{4},kinectWrist{4},errorWrist{4}] = eachItemRunning(trackElbow{4},trackWrist{4},kinectSkeletonData{1,4},order,framelen,kinectOrienData{1,4},origin,la2,21,284,280,300,numW(4));
[kinectElbow{5},errorElbow{5},kinectWrist{5},errorWrist{5}] = eachItemRunning(trackElbow{5},trackWrist{5},kinectSkeletonData{1,5},order,framelen,kinectOrienData{1,5},origin,la2,21,209,210,225,numW(5));
[kinectElbow{6},errorElbow{6},kinectWrist{6},errorWrist{6}] = eachItemRunning(trackElbow{6},trackWrist{6},kinectSkeletonData{1,6},order,framelen,kinectOrienData{1,6},origin,la2,43,224,20,40,numW(6));
[kinectElbow{7},errorElbow{7},kinectWrist{7},errorWrist{7}] = eachItemRunning(trackElbow{7},trackWrist{7},kinectSkeletonData{1,7},order,framelen,kinectOrienData{1,7},origin,la2,46,287,290,310,numW(7));
[kinectElbow{8},errorElbow{8},kinectWrist{8},errorWrist{8}] = eachItemRunning(trackElbow{8},trackWrist{8},kinectSkeletonData{1,8},order,framelen,kinectOrienData{1,8},origin,la2,33,281,280,300,numW(8));
[kinectElbow{9},errorElbow{9},kinectWrist{9},errorWrist{9}] = eachItemRunning(trackElbow{9},trackWrist{9},kinectSkeletonData{1,9},order,framelen,kinectOrienData{1,9},origin,la2,34,278,280,295,numW(9));
[kinectElbow{10},errorElbow{10},kinectWrist{10},errorWrist{10}] = eachItemRunning(trackElbow{10},trackWrist{10},kinectSkeletonData{1,10},order,framelen,kinectOrienData{1,10},origin,la2,29,251,350,370,numW(10));
[kinectElbow{11},errorElbow{11},kinectWrist{11},errorWrist{11}] = eachItemRunning(trackElbow{11},trackWrist{11},kinectSkeletonData{1,11},order,framelen,kinectOrienData{1,11},origin,la2,34,219,20,30,numW(11));
[kinectElbow{12},errorElbow{12},kinectWrist{12},errorWrist{12}] = eachItemRunning(trackElbow{12},trackWrist{12},kinectSkeletonData{1,12},order,framelen,kinectOrienData{1,12},origin,la2,28,207,210,230,numW(12));
[kinectElbow{13},errorElbow{13},kinectWrist{13},errorWrist{13}] = eachItemRunning(trackElbow{13},trackWrist{13},kinectSkeletonData{1,13},order,framelen,kinectOrienData{1,13},origin,la2,27,208,210,240,numW(13));
[kinectElbow{14},errorElbow{14},kinectWrist{14},errorWrist{14}] = eachItemRunning(trackElbow{14},trackWrist{14},kinectSkeletonData{1,14},order,framelen,kinectOrienData{1,14},origin,la2,41,378,380,410,numW(14));

errorElbowOverall = [];
errorWristOverall = [];
for i = 1:N
    errorElbowOverall = [errorElbowOverall,errorElbow{i}];
    errorWristOverall = [errorWristOverall,errorWrist{i}];
end

median__error_elbow = median(errorElbowOverall)
median__error_wrist = median(errorWristOverall)
mean__error_elbow = mean(errorElbowOverall)
mean__error_wrist = mean(errorWristOverall)
realtime_factor_ = mean(realtimeFactor)