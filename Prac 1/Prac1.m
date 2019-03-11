pb = PiBot('172.19.232.105','172.19.232.186',6);

% pb.setVelocity(60,40)


% x1 = 1;
% y1 = 1;
% theta = -45;
% PoseTriangle = poseToTriangle(x1,y1, theta)
% 
% figure
% hold on;
% % line([x1,x2], [y1, y2], 'Color', 'r');
% % line([x2,x3], [y2, y3], 'Color', 'r');
% % line([x3,x1], [y3, y1], 'Color', 'r');
% 
% fill(PoseTriangle(:,1),PoseTriangle(:,2),'r');
% axis([0,2,0,2]);

%%
RobotInfo = pb.getLocalizerPose();
x = RobotInfo.pose.x;
y = RobotInfo.pose.y;
pastPoints = zeros(11,2);
%%
n=1;
while true
    
    RobotInfo = pb.getLocalizerPose();
    x = RobotInfo.pose.x;
    y = RobotInfo.pose.y;
    theta = RobotInfo.pose.theta;
    PoseTriangle = poseToTriangle(x,y, -theta);
    
    pastPoints(1) = [];
    pastPoints = [pastPoints; x y];
    
    hold on

    fill(PoseTriangle(:,1),PoseTriangle(:,2),'r');
    axis([0,2,0,2]);
    
    plot(pastPoints(:,1), pastPoints(:,2), 'b');
    
    hold off
    pause(0.5);
    n = n+1;
    clf
    
end