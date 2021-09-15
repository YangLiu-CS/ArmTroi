function [rotWatchWorld] = calculateRotWatchWorld(gameRotWatch)
% ***************functionality***************:
% calculate the rotation matrix from the watch coordinate system to the world coordinate system
% ***************input***************:
% gameRotWatch: the sensor data from gamerotationvector
% ***************output***************:
% rotWatchWorld: rotation matrix from the watch coordinate system to the world coordinate system

% using GAME_ROTATION_VECTOR to obtain rotWatchWorld in world coordinate system (input: (x,y,z,w))
q = gameRotWatch(:,2:5)';
N = size(q,2);
rotWatchWorld = zeros(3,3,N); % rotWatchWorld in world coordinate system
% from quaternions to rotaion matrix
for i = 1:N
    % the rotation matrix from the watch coordinate system to the world coordinate system
    rotWatchWorld(:,:,i) = [(q(1,i))^2-(q(2,i))^2-(q(3,i))^2+(q(4,i))^2,2*(q(1,i)*q(2,i)-q(4,i)*q(3,i)),2*(q(1,i)*q(3,i)+q(4,i)*q(2,i));
               2*(q(1,i)*q(2,i)+q(4,i)*q(3,i)),-(q(1,i))^2+(q(2,i))^2-(q(3,i))^2+(q(4,i))^2,2*(q(2,i)*q(3,i)-q(4,i)*q(1,i));
               2*(q(3,i)*q(1,i)-q(4,i)*q(2,i)),2*(q(3,i)*q(2,i)+q(4,i)*q(1,i)),-(q(1,i))^2-(q(2,i))^2+(q(3,i))^2+(q(4,i))^2];
end

end