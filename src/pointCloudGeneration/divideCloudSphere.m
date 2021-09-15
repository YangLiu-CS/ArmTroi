function [firstPointCloud,secondPointCloud,firstPointCloudBoundary] = divideCloudSphere(originalPointCloud,m,n,origin,r)
% ***************functionality***************:
% generate the first-layer and second-layer search space for each original point cloud
% ***************input***************:
% originalPointCloud: the original point cloud on a sphere
% m: the granularity of the theta in polar coordinate (r, theta, phi)
% n: the granularity of the phi in polar coordinate (r, theta, phi)
% origin: the coordinate of the shoudler point in rectangular coordinate
% r: the radius of polar coordinate
% ***************output***************:
% firstPointCloud: centers of each block after the original point cloud is divided (used for the second-layer search)
% secondPointCloud: each center (key) corresponds to all possible locations in the block (keyValue) (used for the second-layer search)
% firstPointCloudBoundary: the boundary of each block in the 1st-layer search

num = size(originalPointCloud,2);
originalPointCloud = bsxfun(@minus, originalPointCloud, origin.'); % take the shoulder as the origin
% establish the polar coordinates of each point in the point cloud, the first row is radius r, the second row is latitude theta, the third row is longitude phi
originalPointCloudPhere = zeros(3,num); % polar coordibate
for i = 1:num
    x = originalPointCloud(1,i);
    y = originalPointCloud(2,i);
    z = originalPointCloud(3,i);
    % convert each point in the point cloud to polar form
    originalPointCloudPhere(:,i) = coordinateConversion([x,y,z], r); % polar coordinates with the shoulder as the origin
    r = originalPointCloudPhere(1,i);
    theta = originalPointCloudPhere(2,i);
    phi = originalPointCloudPhere(3,i);
    if (~(isreal(r))) || (~(isreal(theta))) || (~(isreal(phi)))
        mout = 1 % output
    end
end

% the latitude direction is divided into m blocks, and the longitude direction is divided into n blocks
meanTheta = (pi-0)/m;
meanPhi = (2*pi-0)/n;
theta = 0:meanTheta:pi;
phi = 0:meanPhi:2*pi;

firstPointCloud = zeros(3,(length(theta)-1)*(length(phi)-1)); % divide the point cloud into blocks for the first-layer seach using the center of each block only
secondPointCloud = containers.Map; % save all possible locations corresponding to each center with the map data structure for the second-layer search
firstPointCloudBoundary = zeros(4,(length(theta)-1)*(length(phi)-1)); % divide the point cloud into blocks and record the boundary (2 for latitude; 2 for longitude)
k = 1;
q = 1;
for i = 1:length(theta)-1
    for j = 1:length(phi)-1
        % calculate the range of each block
        th1 = theta(i);
        th2 = theta(i+1);
        ph1 = phi(j);
        ph2 = phi(j+1);
        q = q+1;
        keyValue = [];
        true = 0;
        for l = 1:size(originalPointCloudPhere,2)
            if (originalPointCloudPhere(2,l)>=th1) && (originalPointCloudPhere(2,l)<=th2) && (originalPointCloudPhere(3,l)>=ph1) && (originalPointCloudPhere(3,l)<=ph2)
                append = originalPointCloud(:,l)+origin'; % possible locations within the block with the torso as the origin in rectangular coordinate
                keyValue = [keyValue,append];
                true = 1;
            end
        end
        if true == 1 % once the value of true becomes 1, it means that this block should be searched, and its center is saved for the first level search
            tmpth = theta(i) + (theta(i+1)-theta(i))/2;
            tmpph = phi(j) + (phi(j+1)-phi(j))/2;
            firstPointCloud(:,k) = [r*sin(tmpth)*cos(tmpph);r*sin(tmpth)*sin(tmpph);r*cos(tmpth)]; % block center in rectangular coordinate with the shoulder as the origin
            key = matrix2str(firstPointCloud(:,k)+origin'); % block center in rectangular coordinate with the torso as the origin is used as the key
            secondPointCloud(key) = keyValue; % possible locations within the block with the torso as the origin in rectangular coordinate is used as the keyValue
            firstPointCloudBoundary(:,k) = [th1;th2;ph1;ph2]; % block boundary in polar coordinate with the shoulder as the origin
            k = k+1;
        end
    end
end
firstPointCloud = bsxfun(@plus, firstPointCloud, origin.'); % block center in rectangular coordinate with the torso as the origin
firstPointCloud(:,k:(length(theta)-1)*(length(phi)-1)) = [];
firstPointCloudBoundary(:,k:(length(theta)-1)*(length(phi)-1)) = [];
end