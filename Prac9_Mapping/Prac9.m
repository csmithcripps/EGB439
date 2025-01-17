%% Prac 9
clear variables
close all
addpath('../Robot_Functions')
disp('REEEEEEE')
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
tic
while (t < 25)    
    %% Estimate Position   
    % Update Encoder
    encoder = pb.getEncoder;
    dTicks = encoder - prevEncoder;
    prevEncoder = encoder;
    
    cam = pb.getImage();
    dq = encoderToPose(dTicks, q);
    q = q + dq;
    qList = [qList; q];
    
    %% Control Robot
    % Drive toward goal
%     vel = control.driveToPoint(q,goal,0.3,0.10);  % compute the wheel speeds given the current configuration
    vel = [19,20];
    pb.setVelocity(vel)    
    
    
    
    %% Look for Beacons
    idList = idBeacon(cam);
    if ~isempty(idList)
        idList
    end
    for i = 1:size(idList,1)
        [x,y] = getBeaconPos(idList(i,2),idList(i,3), q);
        if idList(i,1) == 27
            colour = 'r*';
        elseif idList(i,1) == 57
            colour = 'b*';
        elseif idList(i,1) == 30
            colour = 'g*';
        elseif idList(i,1) == 39
            colour = 'm*';
        elseif idList(i,1) == 45
            colour = 'k*';
        end
        hold on
        plot(x,y,colour)
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
    t = toc;
end
pb.stop




%% End Program
disp('Press <Enter> to exit')
pause
close all
