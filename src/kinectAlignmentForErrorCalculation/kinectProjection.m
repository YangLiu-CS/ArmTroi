function [kinectElbowBodySphere] = kinectProjection(kinectElbowBody,origin,la2)
% ***************functionality***************:
% this function is used to project the kinectElbow on the estimated sphere
% ***************input***************:
% kinectElbowBody: location sequences of elbow in the torso coordinate system
% origin: the estimated origin coordinate of the sphere in the torso coordinate system
% la2: the radius of the estimated sphere
% ***************output***************:
% kinectElbowBodySphere: projected location sequences of elbow in the torso coordinate system

kinectElbowBodySphere = zeros(size(kinectElbowBody,1),3);
for i = 1:size(kinectElbowBody,1)
    [kinectElbowBodySphere(i,:),~] = sphere_line(origin,la2,kinectElbowBody(i,:));
end
end

function [x2,x3] = sphere_line(x0,r,x1)
% ***************functionality***************:
% this function is used to calculate the intersection of the line from origin to the elbow location from kinect with the sphere
% ***************input***************:
% x0: the estimated origin coordinate of the sphere
% r: the radius of the estimated sphere
% x1: the elbow location
% ***************output***************:
% x2: the intersection in the same direction from origin to the elbow location
% x3: the intersection in the opposite direction

d = 1/norm(x1-x0)*(x1-x0);
x2 = x0 + r * d ;
x3 = x0 - r *  d ;
if nargout==1
    x2 = [x2(:)';x3(:)'] ;
end
end