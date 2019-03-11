function [outputArg1,outputArg2] = drawTriangle(x,y,theta)
%DRAWTRIANGLE 

triangle = poseToTriangle(x,y,theta);
fill(triangle(:,1),triangle(:,2),'b')

end

