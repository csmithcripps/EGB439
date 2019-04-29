%% Prac 7
clear variables
close all
addpath('~/Documents/EGB439/Robot_Functions')

% Init Pibot
pb = PiBot('172.19.232.178');

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

while (1)    
    %% Estimate Position   
    % Update Encoder
    encoder = pb.getEncoder;
    dTicks = encoder - prevEncoder;
    prevEncoder = encoder;
    
    % Update values
    dq = estimateEncoder(dTicks, q);
    if dq == [0 0 0]
        continue
    end
    q = q + dq;
    qList = [qList; q];    
    
    %% Plot
    for qi = qList
        hold on
        drawFrame(qi)
        drawnow
    end
    axis equal
    drawnow
    
    
    %% Control Robot
    % Drive toward goal
    vel = control.driveToPoint(goal, q);
    pb.setVelocity(vel)
    
    % Check if reached goal
    if abs(sum(vel)) < 10
        break
    end
    
    pause(0.1);   
end



%% End Program
pb.stop
pause
close all