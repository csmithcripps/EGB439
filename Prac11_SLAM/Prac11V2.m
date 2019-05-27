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

Q = diag([0.15 4*pi/180]).^2;

mu =   [0;0;0*pi/180];
S =diag([0.1 0.1 0.1*pi/180]).^2;


idList = [];

%% Search the map
disp("REEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
PointList = [1,     0.5;
             1.5,   1;
             1,     1.5;
             0.5,   1];
while (length(idMap)<5)
    for i = 1:length(PointList)
        posX = PointList(i,1);
        posY = PointList(i,2);
        disp("Going to point")
        disp(i)
        [mu,S,idMap] = SLAM2Point(mu,S,idMap,[posX,posY],pb,R,Q);
        if (length(idMap)>=5)
            break
        end
        
    end
end

%% Calculate the centroid
disp("Going To Center")
[Centroidx,Centroidy] = findCentroid(mu,S);

[mu,S,idMap] = SLAM2Point(mu,S,idMap,[Centroidx,Centroidy],pb,R,Q);













