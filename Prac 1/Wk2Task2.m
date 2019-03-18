pb = PiBot('172.19.232.105','172.19.232.12','6');


point = [1,1];


RobotInfo = pb.getLocalizerPose();
x = RobotInfo.pose.x;
y = RobotInfo.pose.y;
theta = RobotInfo.pose.theta;

pastPoints = zeros(11,2);

desiredtheta = atan2((1 - y),(1-x));

deltatheta = desiredtheta - theta;
while deltatheta > 180
    deltatheta = deltatheta - 180;
end
while deltatheta < -180
    deltatheta = deltatheta + 180;
end

end