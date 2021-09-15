factor_times = 1;
factor = pi/20;
m = 30;
n = 2*m;
[num] = testNum(factor);

[pointCloud] = pointCloudsElbowGeneration(ltrunk,la1,la2,la3,factor_times,factor,num);
[pointCloudFirst,pointCloudSecond,pointCloudFirstBoundary] = pointCloudsHierarchicalSearch(pointCloud,m,n,origin,la2,size(pointCloud,1));
clear pointCloud