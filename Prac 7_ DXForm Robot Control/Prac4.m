%% Prac 4
close all

pb = PiBot('172.19.232.178','172.19.232.12',4);

% Simulator mode
SIMULATION = true;

% Create path
A = 0.5/sqrt(2);
n = 40;
t = linspace(-0.5*pi, 1.5*pi, n/2);
xt = (A*sqrt(2)*cos(t))./(sin(t).^2+1) + 1;
xt = [xt xt];
yt = 1.7 * (A*sqrt(2).*cos(t).*sin(t))./(sin(t).^2+1) + 1;
yt = [yt yt];

% Variables
if SIMULATION
    q = [1 1 0];
else
    pose = pb.getLocalizerPose.pose;
    while pose.x == 0 && pose.y == 0 && pose.theta == 0
        pose = pb.getLocalizerPose.pose;
    end
    q = [pose.x pose.y deg2rad(pose.theta)];
end
d = 0.01;
dt = 1;
qs = zeros([n 3]);

% Initial control
goal = [xt(1) yt(1)];
vel = controlP4(goal, q, d, dt, true);

% Loop
if ~SIMULATION; tic; end
for i = 2:n
    
    % Loop delay
    if ~SIMULATION
        while toc < dt
            continue
        end
        disp(toc)
        tic
    end
    
    % Update position
    if SIMULATION
        q = qupdate(q, vel, dt);
    else
        pose = pb.getLocalizerPose.pose;
        while pose.x == 0 && pose.y == 0 && pose.theta == 0
            pose = pb.getLocalizerPose.pose;
        end
        q = [pose.x pose.y deg2rad(pose.theta)];
    end
    qs(i-1,:) = q;
    
    % Control the robot
    goal = [xt(i) yt(i)];
    vel = controlP4(goal, q, d, dt, true);
    if ~SIMULATION
        pb.setVelocity(vel)
    end

    % Live plot
    qplot(q);
    hold on
    plot(xt, yt, 'r-')
    hold on
    plot(goal(1), goal(2), 'kp')
    hold on
    plot(qs(1,1), qs(1,2), 'k.')
    plot(qs(:,1), qs(:,2))
    hold on
    xlim([0 2])
    ylim([0 2])
    drawnow
    
end