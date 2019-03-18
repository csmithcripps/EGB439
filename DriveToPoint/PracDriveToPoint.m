pb = PiBot('172.19.232.178','172.19.232.11',6);


point = [1 1];
dt = 0.2;  % sample interval
for step = 1:200
  q = piBotHelpers.getPoseVector(pb);% update the configuration for one timestep  
  if q == [0,0,0]
      vel = [0,0];
  else
      vel = control.driveToPoint(q,point,0.2,0.15)  % compute the wheel speeds given the current configuration
  end
  
  pb.setVelocity(vel);
  
  piBotHelpers.qplot(q,point);  % display the robot's new configuration
  
  pause(dt)
  pb.stop
%   pause(dt/10 )
  clf
end