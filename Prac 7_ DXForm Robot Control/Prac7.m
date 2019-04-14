
%% Prac 7

clear all
addpath('../Robot_Functions')

%% Init Variables

SIM = false;
USING_BOT = false;
goal = [0.1,0.1];
scalepx = 100/2;
n = 40;
d = 0.01;
dt = 0.25;
qs = zeros([n 3]);

%% Access PiBot


if ~SIM
    pb = PiBot('172.19.232.178','172.19.232.11',6);
    disp('1. Getting Robot Info')
    img = pb.getLocalizerImage();

else
    %SIMULATED VARIABLES
    disp('1. Setting Virtual Robot Variables')
    img = zeros(500,500);
    img(200:300,100:250) = 100;
    img(250:300,120:500) = 100;
end
%% Compute Occupancy Grid

disp('2. Making Map')
disp('.... a: Thresholding for obstacles')

imgThreshhold = img > 20;
imgThreshhold(:,1:20) = 0;
imgThreshhold(:,end-20:end) = 0;
imgThreshhold(1:20,:) = 0;
imgThreshhold(end-20:end,:) = 0;

occupancyGrid = imresize(imgThreshhold, 1/5);
% 
% %Remove Robot
% if x ~= 0 && y ~= 0
%     disp('.... b: Removing Robot from occupancy grid')
%     x1   = round(max(x*50 - 10,1));
%     y1   = round(min(y*50 - 10,100));
%     x2   = round(max(x*50 + 10,1));
%     y2   = round(min(y*50 + 10,100));
%     occupancyGrid(y1:y2,x1:x2) = 0;
% end

%% Distance Transform

disp('3. Analysing Map')
    
goalpx = goal * scalepx;

% Enlarge Obstacles by 5 to allow space for the robot
se = strel('disk',5);
expandedMap = imdilate(occupancyGrid,se);
expandedMap = flipud(expandedMap);

disp('.... a: Computing Dist Transform')
tic
assert(~isnan(expandedMap(goalpx(2),goalpx(1))));
dtransform = PathPlanning.distanceTransform(double(expandedMap), goalpx);
toc
%% Path Planning
disp("!!--Place Robot and Press Enter--!!")
pause;

if USING_BOT
    pose = pb.getLocalizerPose.pose;
    x = pose.x;
    y = 2-pose.y;
    theta = pose.theta;
    disp(['....x = ',x])
    disp(['....y = ',y])
else
    x = 1.9;
    y = 1.9;
    theta = -pi/2;    
end

start = [x,y] * scalepx;
start = [round(start(1)),round(start(2))];
assert(~isnan(expandedMap(start(2),start(1))));

disp('.... b: Computing Path')
% 1 at the end means that the dtransform was already computed (doesn't recompute)
tic
path = PathPlanning.findPath(double(dtransform), start, goalpx,1); 
toc

%% Display Output

disp('4. Creating Graphical Representation')

RGB = zeros(100,100,3);
RGB(:,:,1)   = occupancyGrid;
RGB(:,:,2)   = ones(100,100) - occupancyGrid;
pathDisp = 2*path/100;

figure(1)
subplot(111)
expandedMap = flipud(expandedMap);
idisp (expandedMap,'xydata',{[0,2] [2,0]},'ynormal')
hold on
plot(pathDisp(:,1),pathDisp(:,2))
hold off

figure(2)
subplot(211)
idisp (dtransform,'xydata',{[0,2] [2,0]},'ynormal')
hold off

figure(3)
idisp (RGB,'xydata',{[0,2] [2,0]},'ynormal')
hold on
plot(pathDisp(:,1),pathDisp(:,2))
title("Path Required")
hold off
%% Pure Pursuit
if SIM 
    
    figure(4)
    rob = robotSim([x, y, theta],dt);
    first = 1;
    tic
    for point = pathDisp.'    
        figure(3)
        hold on
        idisp (RGB,'xydata',{[0,2] [2,0]},'ynormal', 'nogui');    
        vel = control.purePursuit(point, rob.q, d, dt, first);  % compute the wheel speeds given the current configuration
        rob.update(vel);
        hold on     
        rob.plot()
        hold on
        plot(pathDisp(:,1),pathDisp(:,2),'b--')
        hold on
        scatter(point(1),point(2),'rp') 
        first = 0;
        pause(0.01)
    end
    toc
    hold on
    rob.plot()
    plot(pathDisp(:,1),pathDisp(:,2),'b--')
end

        
if USING_BOT
%     % Initial control
%     q = [pose.x pose.y deg2rad(pose.theta)];
%     goal = [goalpx(2) goalpx(1)];
%     vel = controlP4(goal, q, d, dt, true);
%     for i = 1:size(path,1)
%         pose = pb.getLocalizerPose.pose;
%         while pose.x == 0 && pose.y == 0 && pose.theta == 0
%             pose = pb.getLocalizerPose.pose;
%         end
%         q = [pose.x pose.y deg2rad(pose.theta)];
%         
%         % Control the robot
%         goal = [path(i) path(i)];
%         vel = controlP4(goal, q, d, dt, true);
%         if ~SIM
%             pb.setVelocity(vel)
%         end
%         pause(dt)
%     end
    
    
    figure(4)
    first = 1;
    tic
    for point = pathDisp.'   
        pose = pb.getLocalizerPose.pose;
        while pose.x == 0 && pose.y == 0 && pose.theta == 0
            pose = pb.getLocalizerPose.pose;
        end
        q = [pose.x pose.y deg2rad(pose.theta)]; 
        figure(3)
        hold on
        idisp (RGB,'xydata',{[0,2] [2,0]},'ynormal', 'nogui');    
        vel = control.purePursuit(point, q, d, dt, first);  % compute the wheel speeds given the current configuration
        
        hold on     
        piBotHelpers.plot(q)
        hold on
        plot(pathDisp(:,1),pathDisp(:,2),'b--')
        hold on
        scatter(point(1),point(2),'rp') 
        first = 0;
        pause(dt)
    end
    toc
    hold on
    rob.plot()
    plot(pathDisp(:,1),pathDisp(:,2),'b--')
end

if ~SIM && ~USING_BOT  
    while(1)
        pose = pb.getLocalizerPose.pose;
        x = pose.x;
        y = pose.y;
        dist = sqrt((x-goal(1))^2 + (y-goal(2))^2);
        if dist > 1.5
            %First row
        elseif dist > 1
            %Second Row
        elseif dist > 0.5
            %Third Row
        elseif dist < 0.25
            %ALL LEDS
        end
            
    end
end


pb.stop;
