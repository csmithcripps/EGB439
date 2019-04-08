clear all
addpath('~/Documents/EGB439/Robot_Functions')
Rob = PiBot('172.19.232.104','172.19.232.12',6);

dt = 0.2;
q = piBotHelpers.getPoseVector(Rob);
lineVar = [1,1,-2];
piBotHelpers.qplot(q)
for step = 1:500
    q = piBotHelpers.getPoseVector(Rob);
    if q == [0,0,0]
        vel = [0,0]
    else
        vel = control.AlongLine(q,lineVar,0.2,0.4)  % compute the wheel speeds given the current configuration(
    end
    
    Rob.setVelocity(vel);
    
    % Plot
    piBotHelpers.qplot(q);  % display the robot's new configuration
    x = 0:0.1:2;
    y = -(lineVar(1)*x + lineVar(3))/lineVar(2);
    hold on
    plot(x,y,'r--')
    axis 'square'
    hold off   
    
    
    pause(dt)
%     Rob.stop
    clf
end