function [pointCloud] = pointCloudsElbowGeneration(ltrunk,la1,la2,la3,factor_times,factor,num)
% ***************functionality***************:
% generate original point clouds for each wrist orientation (taking the left arm as an example)
% ***************input***************:
% ltrunk: the length of the trunk
% la1: the length of the half shoulder
% la2: the length of the upper arm
% la3: the length of the lower arm
% factor_times: tolZ = factor_times*factor; tolX = factor_times*factor; tolY = factor_times*factor
% factor: the granularity to divide the 5 DoFs for point cloud generation
% num: the number of overall elbow locations based on the factor
% ***************output***************:
% pointCloud: the generated point clouds: key--the euler angle; keyValue--the possible elbow locations

% arm pointCloud generation: the range of each degree of freedom
% theta1: -pi~1/3*pi
% theta2: -2/9*pi~2/3*pi
% theta3: -2/3*pi~1/6*pi
% theta4: -5/6*pi~0
% theta5: -pi~0
i = 0;
% only generate the point cloud of the elbow for HMM search
rotWatchLabel = zeros(3,2,num);
tolZ = factor_times*factor;
tolX = factor_times*factor;
tolY = factor_times*factor;
for theta1 = -pi:factor:1/3*pi
    for theta2 = -2/9*pi:factor:2/3*pi
        for theta3 = -2/3*pi:factor:1/6*pi
             for theta4 = -5/6*pi:factor:0
                 for theta5 = -pi:factor:0
                    [~, ~, locElbow, ~, rotWrist] = mathArmModel(theta1,theta2,theta3,theta4,theta5,ltrunk,la1,la2,la3);
                    locElbow = locElbow(1:3,4);
                    rotWrist = rotWrist(1:3,1:3);
                    
                    thetaZ = atan2(-rotWrist(1,2),rotWrist(2,2));
                    thetaX = asin(rotWrist(3,2));
                    thetaY = atan2(-rotWrist(3,1),rotWrist(3,3));
                    thetaZ = round(thetaZ/tolZ)*tolZ;
                    thetaX = round(thetaX/tolX)*tolX;
                    thetaY = round(thetaY/tolY)*tolY;
                    
                    i = i+1;
                    rotWatchLabel(:,:,i)  = [[thetaX;thetaY;thetaZ],locElbow]; % three rows and two columns, the first column: euler angles, the second column: coordinate points, both in the torso coordinate system
                 end
             end
        end
    end
end

% initialize a map object
pointCloud = containers.Map;
for i = 1:num
    label = rotWatchLabel(:,1,i); % the euler angle
    key = matrix2str(label); % the key of map
    if ~(pointCloud.isKey(key))
        keyValue = rotWatchLabel(:,2,i);
        pointCloud(key) =  keyValue;
    else
        originKeyValue = pointCloud(key);
        appendKeyValue = rotWatchLabel(:,2,i);
        newKeyValue = [originKeyValue,appendKeyValue];
        pointCloud(key) = newKeyValue;
    end
    % output the i
    if mod(i,10000) == 0
        i
    end
end

end