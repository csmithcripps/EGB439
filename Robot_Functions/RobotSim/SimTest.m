q = [0.5 0.5 pi]; % in the middle of the arena facing right
point = [1 1];
dt = 0.1;  % sample interval
for step = 1:200
  vel = control.driveToPoint(q,point,6,0.6);  % compute the wheel speeds given the current configuration
  q = piBotHelpers.qupdate(q, vel, dt);  % update the configuration for one timestep
  piBotHelpers.qplot(q,point);  % display the robot's new configuration
  pause(dt)
  clf
end

% dt = 0.1;
% Rob = robotSim([0.5 0.5 45], dt);
% lineVar = [-1,1,2];
% for step = 1:500
%   vel = control.AlongLine(Rob.q,lineVar);  % compute the wheel speeds given the current configuration(
%   Rob.update(vel);
%   Rob.plot(lineVar);  % display the robot's new configuration
%   pause(dt)
%   clf
% end