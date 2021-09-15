function [pointCloudFirst,pointCloudSecond,pointCloudFirstBoundary] = pointCloudsHierarchicalSearch(pointCloud,m,n,origin,r,num)
% ***************functionality***************:
% based on the original point clouds, generate the search space of the 1st-layer and 2nd-layer seach
% ***************input***************:
% pointCloud: the generated original point clouds: key--the euler angle; keyValue--the possible elbow locations (on a sphere)
% m: the granularity of the theta in polar coordinate (r, theta, phi) [0, pi]
% n: the granularity of the phi in polar coordinate (r, theta, phi) [0, 2*pi]: thus, n == 2*m to make the block squared
% origin: the coordinate of the shoudler point in rectangular coordinate
% r: the radius of polar coordinate
% num: the number of generated point clouds
% ***************output***************:
% pointCloudFirst: the search space for the 1st-layer search
% pointCloudSecond: the search space for the 2nd-layer seach
% pointCloudFirstBoundary: the boundary of each block in the 1st-layer search

keys = pointCloud.keys;
pointCloudFirst = containers.Map;
pointCloudSecond = containers.Map;
pointCloudFirstBoundary = containers.Map;

for i = 1:num
    key = keys{i};
    keyValue = pointCloud(key);
    % generate the first-layer and second-layer search space for each original point cloud
    [firstPointCloud,secondPointCloud,firstPointCloudBoundary] = divideCloudSphere(keyValue,m,n,origin,r);
    keyValueFirst = firstPointCloud;
    pointCloudFirst(key) = keyValueFirst;
    keyValueSecond = secondPointCloud;
    pointCloudSecond(key) = keyValueSecond;
    keyValueFirstBoundary = firstPointCloudBoundary;
    pointCloudFirstBoundary(key) = keyValueFirstBoundary;
end
end