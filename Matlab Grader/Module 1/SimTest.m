q = [1 0.5 pi]; % in the middle of the arena facing right
point = [1 1];
dt = 0.1;  % sample interval
for step = 1:1000
  vel = control(q,point)  % compute the wheel speeds given the current configuration
  q = qupdate(q, vel, dt);  % update the configuration for one timestep
  qplot(q);  % display the robot's new configuration
  pause(dt)
  clf
end