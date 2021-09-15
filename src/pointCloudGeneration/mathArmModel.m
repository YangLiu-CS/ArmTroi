function [locSwivel,locShoulder,locElbow,locWrist,rotWrist] = mathArmModel(th1,th2,th3,th4,th5,ltrunk,la1,la2,la3)
% This function is used to calculate the locations of skeleton points using the skeleton model

[A1,A2,A3,A4,A5,La1,La2,La3] = matrixArm(th1,th2,th3,th4,th5,la1,la2,la3);
locSwivel = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, ltrunk; 0, 0, 0, 1];
locShouldertoSwivel = La1; % in the swivel coordinate system, the location of shoulder
locShoulder = locSwivel + locShouldertoSwivel;
locElbowtoShoulder = (A1*A2*A3)*La2;
locElbow = locShoulder+locElbowtoShoulder;
% locWristtoElbow = (A1*A2*A3)*(A4*A5*La3);
% A5 cannot influence the location of wrist
locWristtoElbow = (A1*A2*A3)*(A4*La3);
locWrist = locElbow+locWristtoElbow;
rotWrist = A1*A2*A3*A4*A5;

end