%% Prac 9
clear variables
close all
addpath('../Robot_Functions')
disp('REEEEEEE')
% Init Pibot
pb = PiBot('172.19.232.173');

% Initial values/constants
q = [1 1 pi/2];
goal = [0 0];

map = [0.15,0.15;
       0.9,0.15;
       1.83,0.13;
       0.4,1.85;
       1.6,1.85];

% Setup values
qList = q;
prevEncoder = [0 0];

% Setup robot/plot
pb.resetEncoder
figure
hold on
r = sqrt(0.5^2 + 0.5^2);
S = diag([0.001 0.001 1*pi/180]).^2;

R = diag([0.1 10*pi/180]).^2;

Q = diag([0.08 2*pi/180]).^2;
t = 0;
tic

G = [0.61, 0.6, 70*pi/180;
     1.49, 0.58, 120*pi/180];
% GOAL
facing = false;
    while (~facing)
        %% Read Inputs
        encoder = pb.getEncoder;
        cam = pb.getImage();
        cam(1:50,:,:) = 0;
        %% Predict  
        % Update Encoder    
        dTicks = encoder - prevEncoder;
        prevEncoder = encoder;
        dq = encoderToPose(dTicks, q);
        [q,S] = predict_step(q,S,dq,R);

        %% Update
        idList = idBeacon(cam);
        for j = 1:size(idList,1)
            disp('Beacons:')
            disp(idList)
            [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
            [q,S] = update_step(idList(j,1),[r;b],q,S,Q);    
            if idList(j,1) == 39
                facing = true;
            end
        end

        %% Control Robot
        % Drive toward goal
%         vel = control.driveToPoint(q,goal(1:2),0.1,0.08);  % compute the wheel speeds given the current configuration
        vel = [-10,10];
        pb.setVelocity(vel)    

        %% Plot
        clf
        axis([0,2,0,2])
        hold on
        plot_cov(q,S,3)%% Read Inputs
        encoder = pb.getEncoder;
        cam = pb.getImage();
        cam(1:50,:,:) = 0;
        %% Predict  
        % Update Encoder    
        dTicks = encoder - prevEncoder;
        prevEncoder = encoder;
        dq = encoderToPose(dTicks, q);
        [q,S] = predict_step(q,S,dq,R);

        %% Update
        idList = idBeacon(cam);
        for j = 1:size(idList,1)
            disp('Beacons:')
            disp(idList)
            [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
            [q,S] = update_step(idList(j,1),[r;b],q,S,Q);    
            if idList(j,1) == 39
                facing = true;
            end
        end

        %% Control Robot
        % Drive toward goal
%         vel = control.driveToPoint(q,goal(1:2),0.1,0.08);  % compute the wheel speeds given the current configuration
        vel = [-10,10];
        pb.setVelocity(vel)   
        hold on
        drawFrame(q)
        for i = 1:size(map,1)
            hold on
            plot(map(i,1),map(i,2),'rp')
        end
        axis equal
        drawnow
    end
    
    
for i = 1:2
    goal = G(i,:);
    while (r > 0.05)
        %% Read Inputs
        encoder = pb.getEncoder;
        cam = pb.getImage();
        cam(1:50,:,:) = 0;
        %% Predict  
        % Update Encoder    
        dTicks = encoder - prevEncoder;
        prevEncoder = encoder;
        dq = encoderToPose(dTicks, q);
        [q,S] = predict_step(q,S,dq,R);

        %% Update
        idList = idBeacon(cam);
        for j = 1:size(idList,1)
            disp('Beacons:')
            disp(idList)
            [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
            [q,S] = update_step(idList(j,1),[r;b],q,S,Q);  
            if idList(j,1) == 27
                colour = 'r*';
            elseif idList(j,1) == 57
                colour = 'b*';
            elseif idList(j,1) == 30
                colour = 'g*';
            elseif idList(j,1) == 39
                colour = 'm*';
            elseif idList(j,1) == 45
                colour = 'k*';
            end
            hold on
            plot(r*cos(q(3)),b*cos(q(3)),colour)
            drawnow      
        end

        %% Control Robot
        % Drive toward goal
        vel = control.driveToPoint(q,goal(1:2),0.1,0.08);  % compute the wheel speeds given the current configuration
%         vel = [-10,10];
        pb.setVelocity(vel)    

        %% Plot
%         clf
        axis([0,2,0,2])
        hold on
        plot_cov(q,S,3)
        hold on
        drawFrame(q)
        for i = 1:size(map,1)
            hold on
            plot(map(i,1),map(i,2),'rp')
        end
        axis equal
        drawnow


        r = sqrt((goal(1) - q(1))^2 + (goal(2) - q(2))^2);
        t = toc;
        pb.stop()
        pause(0.1)
    end
    %% TODO Turn to Angle
    
    %Wait 5s
    tic
    t=0;
    while (t<5)
        t = toc;
    end
end
pb.stop




%% End Program
disp('Press <Enter> to exit')
pause
close all
