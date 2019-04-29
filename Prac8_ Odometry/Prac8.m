%% Prac 7
clear variables
close all
addpath('../Robot_Functions')

% Init Pibot
pb = PiBot('172.19.232.173');

% Initial values/constants
q = [0.1 0.1 0];
goal = [1 1];

% Setup values
qList = q;
prevEncoder = [0 0];

% Setup robot/plot
pb.resetEncoder
figure
hold on
r = sqrt(0.5^2 + 0.5^2);
while (r > 0.3)    
    %% Estimate Position   
    % Update Encoder
    encoder = pb.getEncoder;
    dTicks = encoder - prevEncoder;
    prevEncoder = encoder;
    
    %% Control Robot
    % Drive toward goal
    vel = control.driveToPoint(q,goal,0.3,0.04);  % compute the wheel speeds given the current configuration
    pb.setVelocity(vel)
    
    % Update values
    dq = encoderToPose(dTicks, q);
    if dq == [0 0 0]
        continue
    end
    q = q + dq;
    qList = [qList; q];    
    
    %% Plot
    for i = size(qList,1)
        hold on
        drawFrame(qList(i,:))
        drawnow
    end
    axis equal
    hold on 
    plot(goal(1),goal(2), 'bo')
    drawnow
    
    r = sqrt((goal(1) - q(1))^2 + (goal(2) - q(2))^2);
    if r < 0.4       
        cam = pb.getImage();
        idList = idBeacon(cam);
        if ~isempty(idList)
            idList
        end
        disp(idList)
    end
        
    pause(0.1);   
end
pb.stop

%% Look for Beacons
cam = pb.getImage();
idList = idBeacon(cam);
if ~isempty(idList)
    idList
end


%% End Program
disp('Press <Enter> to exit')
pause
close all