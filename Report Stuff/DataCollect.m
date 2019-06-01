%% Report Data Collection

%Init Variables & Connect to bot
clear variables
close all
addpath('../Robot_Functions')
disp('REEEEEEE')
% Init Pibot
pb = PiBot('172.19.232.108','172.19.232.12',6);
pb.resetEncoder();
prevEncoder = pb.getEncoder();


pb.stop()
pb.resetEncoder()
prevEncoder = 0;

o = [0,0];
z = zeros(5,3);
q = [0,0,0];

k = 1;
tic
t=0;
while(t<60)
    %% Read Inputs
    encoder = pb.getEncoder;
    cam = pb.getImage();
    
    %% Encoder Information   
    dTicks = encoder - prevEncoder;
    prevEncoder = encoder;
    o(k,:) = dTicks;
    
    %% Sensor Information
    a = zeros(5,3);
    idList = idBeacon(cam);
    for j = 1:size(idList,1)
        [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
        a(j,:) = [idList(1),r,b];
    end
    z(:,:,k) = a;
    
    %% Ground Truth (Global Localiser)
    pose = pb.getLocalizerPose.pose;
    if pose.x == 0
        q(k,:) = q(k-1,:);
    else
        q(k,:) = [pose.x,pose.y,deg2rad(pose.theta)];
    end
    
    %% Control Robot
    % Drive toward goal
    vel = [10,15];
    pb.setVelocity(vel)
    
    k=k+1
    t=toc;
end

pb.stop()
save('reportData.mat','o','z','q','k')
disp("SLAMMMED")