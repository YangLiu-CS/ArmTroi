function [A1,A2,A3,A4,A5,La1,La2,La3] = matrixArm(th1,th2,th3,th4,th5,la1,la2,la3)

% 5 rotation matrixs for 5 DoFs
% 3 translation matrixs for 3 limbs
A1 = [cos(th1), 0, sin(th1), 0; 0, 1, 0, 0; -sin(th1), 0, cos(th1), 0; 0, 0, 0, 1];    % y
A2 = [1, 0, 0, 0; 0, cos(th2), -sin(th2), 0; 0, sin(th2), cos(th2), 0; 0, 0, 0, 1];    % x
A3 = [cos(th3), -sin(th3), 0, 0; sin(th3), cos(th3), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];    % z
A4 = [cos(th4), 0, sin(th4), 0; 0, 1, 0, 0; -sin(th4), 0, cos(th4), 0; 0, 0, 0, 1];    % y
A5 = [cos(th5), -sin(th5), 0, 0; sin(th5), cos(th5), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];    % z
La1 = [1, 0, 0, 0; 0, 1, 0, la1; 0, 0, 1, 0; 0, 0, 0, 1]; % left hand is positive
La2 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, -la2; 0, 0, 0, 1];
La3 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, -la3; 0, 0, 0, 1];

end