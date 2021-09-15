function [num] = testNum(factor)
% ***************functionality***************:
% calculate the the number of all rotWrists

i=0;
for theta1 = -pi:factor:1/3*pi
    for theta2 = -2/9*pi:factor:2/3*pi
        for theta3 = -2/3*pi:factor:1/6*pi
             for theta4 = -5/6*pi:factor:0
                 for theta5 = -pi:factor:0
                     i=i+1;
                 end
             end
        end
    end
end
num = i;
end