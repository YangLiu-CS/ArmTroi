function [vp] = coordinateConversion(v,r)
% ***************functionality***************:
% convert the point in the Cartesian (rectangular) coordinate system to the polar coordinate system
% ***************input***************:
% v: the coordinate in the Cartesian coordinate system
% r: the radius
% ***************output***************:
% vp: the coordinate in the Polar coordinate system

x = v(1);
y = v(2);
z = v(3);

theta = acos(z/r);
if ~(isreal(theta))
    theta = asin(sqrt(x^2+y^2)/r);
end

if x<0
    phi = pi + atan(v(2)/v(1));
else
    if y > 0
        phi = atan(v(2)/v(1));
    else
        phi = 2*pi + atan(v(2)/v(1));
    end
end

% convert each point to polar form
vp = [r;theta;phi];

end