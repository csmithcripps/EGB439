
pb = PiBot('172.19.232.178','172.19.232.11',6);
d = 0.01;
% Create path
A = 0.5/sqrt(2);
n = 40;
t = linspace(-0.5*pi, 1.5*pi, n/2);
xt = (A*sqrt(2)*cos(t))./(sin(t).^2+1) + 1;
xt = [xt xt];
yt = 1.7 * (A*sqrt(2).*cos(t).*sin(t))./(sin(t).^2+1) + 1;
yt = [yt yt];


q0 = piBotHelpers.getPoseVector(pb);
x = [q0(1)];
y = [q0(2)];
i = 1;
point = [xt(1) yt(1)];
dt = 0.2;  % sample interval
for step = 1:200
    
    
   
    q = piBotHelpers.getPoseVector(pb);% update the configuration for one timestep  
    x = [x q(1)];
    y = [y q(2)];
    if q == [0,0,0]
      vel = [0,0];
    else
      vel = control.purePursuit(q,point,0.2,0.15,d);  % compute the wheel speeds given the current configuration
    end

    r = sqrt((point(1) - q(1))^2 + (point(2) - q(2))^2);
    if r < (d+0.1)
        i = i+1;
        point = [xt(i) yt(i)];
    end
        
    pb.setVelocity(vel);

    piBotHelpers.qplot(q,point);  % display the robot's new configuration
    hold on
    scatter(q0(1),q0(2),'k');
    plot(x,y)
    pause(dt)
%     pb.stop
    %   pause(dt/10 )
    clf
end