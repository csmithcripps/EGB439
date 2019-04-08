clear all
addpath('~/Documents/EGB439/Robot_Functions')
dt = 0.1;
Rob = robotSim([1.5 1.5 pi], dt);
lineVar = [1,1,-2];
Rob.plot()
for step = 1:500
  vel = control.AlongLine(Rob.q,lineVar,10,15)  % compute the wheel speeds given the current configuration(
  Rob.update(vel);
 
  piBotHelpers.qplot(Rob.q);  % display the robot's new configuration
  x = 0:0.1:2;
  y = -(lineVar(1)*x + lineVar(3))/lineVar(2);
  hold on
  plot(x,y,'r--')
  hold off
  pause(dt)
  clf
end