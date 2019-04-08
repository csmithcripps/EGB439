function [PoseTriangle] = poseToTriangle(x,y,theta)
%POSETOTRIANGLE Converts an input pose to a triangle plotted
l = 0.1;
w = 0.04;
theta = -theta;
R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
triangle = [0 0; -l +w; -l -w];

PoseTriangle = triangle*R + [x,y; x,y; x,y];



end

