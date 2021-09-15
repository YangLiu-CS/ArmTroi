function [rotMatrix] = rotMatrixCal(q)
% ***************functionality***************:
% this function is used convert quaternion to rotaion matrix
% ***************input***************:
% q: the quaternion (x,y,z,w)
% ***************output***************:
% rotMatrix: the calculated rotaion matrixs

N = size(q,2);
rotMatrix = zeros(3,3,N);
for i = 1:N
    rotMatrix(:,:,i) = [(q(1,i))^2-(q(2,i))^2-(q(3,i))^2+(q(4,i))^2,2*(q(1,i)*q(2,i)-q(4,i)*q(3,i)),2*(q(1,i)*q(3,i)+q(4,i)*q(2,i));
                        2*(q(1,i)*q(2,i)+q(4,i)*q(3,i)),-(q(1,i))^2+(q(2,i))^2-(q(3,i))^2+(q(4,i))^2,2*(q(2,i)*q(3,i)-q(4,i)*q(1,i));
                        2*(q(3,i)*q(1,i)-q(4,i)*q(2,i)),2*(q(3,i)*q(2,i)+q(4,i)*q(1,i)),-(q(1,i))^2-(q(2,i))^2+(q(3,i))^2+(q(4,i))^2];
end

end