clear all
load('purepursuit.mat')
addpath('~/Documents/EGB439/Robot_Functions')
d = 0.1;
dt = 0.25;

rob = robotSim([0,0, 0],dt);
first = 1;
for point = robot_traj.'
    vel = control.purePursuit(point, rob.q, d, dt, first);  % compute the wheel speeds given the current configuration
    rob.update(vel);
    hold on
    rob.plot();
    plot(robot_traj(:,1),robot_traj(:,2),'r--')
    scatter(point(1),point(2),'rp')
    hold off
    pause(0.01)
    clf
    
    
    first = 0;
    
end

hold on
rob.plot()
plot(robot_traj(:,1),robot_traj(:,2),'r--')
    