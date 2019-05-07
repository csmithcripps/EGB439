%% Prac 9
clear variables
close all
addpath('../Robot_Functions')

% Init Pibot
pb = PiBot('172.19.232.173');

% Initial values/constants
q = [0 0 0];
goal = [1 1];

% Setup values
qList = q;
prevEncoder = [0 0];

% Setup robot/plot
pb.resetEncoder
figure
hold on
r = sqrt(0.5^2 + 0.5^2);
t = 0;
while (t < 20)    
    %% Estimate Position   
    % Update Encoder
    encoder = pb.getEncoder;
    dTicks = encoder - prevEncoder;
    prevEncoder = encoder;
    
    dq = encoderToPose(dTicks, q);
    q = q + dq;
    qList = [qList; q];
    
    %% Control Robot
    % Drive toward goal
%     vel = control.driveToPoint(q,goal,0.3,0.10);  % compute the wheel speeds given the current configuration
    vel = [15,20];
    pb.setVelocity(vel)    
    
    
    
    %% Look for Beacons
    cam = pb.getImage();
    idList = idBeacon(cam);
    for i = 1:size(idList,1)
        [x,y] = getBeaconPos(idList(i,2),idList(i,3), q);
        hold on
        plot(x,y,'k*')
        drawnow
    end
    %% Plot
    for i = size(qList,1)
        hold on
        drawFrame(qList(i,:))
        drawnow
    end
    axis equal
    drawnow
    
    
    r = sqrt((goal(1) - q(1))^2 + (goal(2) - q(2))^2);
    pause(0.1); 
    t = t + 0.1;
end
pb.stop




%% End Program
disp('Press <Enter> to exit')
pause
close all