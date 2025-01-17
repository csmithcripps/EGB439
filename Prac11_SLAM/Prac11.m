%% Prac 11 SLAM 

%Init Variables & Connect to bot
clear variables
close all
addpath('../Robot_Functions')
disp('REEEEEEE')
% Init Pibot
pb = PiBot('172.19.232.173');
pb.resetEncoder();
prevEncoder = 0;
idMap = [];


R = diag([0.1 10*pi/180]).^2;

Q = diag([0.08 2*pi/180]).^2;

mu =   [0;0;0*pi/180];
S =diag([0.1 0.1 0.1*pi/180]).^2;


idList = [];
%% Identify first beacon

while (isempty(idList))
    
    vel = [10,-10];
    pb.setVelocity(vel);   
    pause(0.2)
    pb.stop()
    pause(0.05)
    
    cam = pb.getImage();
    idList = idBeacon(cam);  
     
    
end

pb.stop();
% Reinitialise Variables
disp('REEEEEEEEEEEEEEEEEEEEEEEEE')
mu = [0;0;0];

%find first goal location
[r,beta] = getBeaconRangeBearing(idList(1,2),idList(1,3));

posR = r-0.2;
posX = posR*sin(beta);
posY = posR*cos(beta);

%% SLAM to Point 1
[mu,S,idMap] = SLAM2Point(mu,S,idMap,[posX,posY],pb,R,Q);

%% Search the map
i = 2;
while (length(idMap)<5)
    disp("Finding Beacon ")
    disp(i)
    [posX,posY] = curiousBot(2,mu,idMap,pb,S,R,Q);
    
    disp("Going to point")
    disp(i)
    [mu,S,idMap] = SLAM2Point(mu,S,idMap,[posX,posY],pb,R,Q);
    
    i=i+1;
end

%% Calculate the centroid
[Centroidx,Centroidy] = findCentroid(mu,S);

[mu,S,idMap] = SLAM2Point(mu,S,idMap,[Centroidx,Centroidy],pb,R,Q);













