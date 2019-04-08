%% Prac 7

clear all
addpath('../Robot_Functions')

%% Init Variables

SIM = false;
USING_BOT = false;
goal = [0.5,0.5];
scalepx = 100/2;

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
%%
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
disp("!!--Place Robot and Press Enter--!!")
pause;
%% Find Path
%%
disp('3. Analysing Map')

if ~SIM && USING_BOT
    pose = pb.getLocalizerPose.pose;
    x = pose.x;
    y = 2-pose.y;
    theta = pose.theta;
    disp(['....x = ',x])
    disp(['....y = ',y])
else
    x = 1.5;
    y = 1.5;
    theta = -pi/2;    
end
    
start = [x,y] * scalepx;
goalpx = goal * scalepx;

% Enlarge Obstacles by 5 to allow space for the robot
se = strel('disk',5);
expandedMap = imdilate(occupancyGrid,se);
expandedMap = flipud(expandedMap);

disp('.... a: Computing Dist Transform')
tic
dtransform = PathPlanning.distanceTransform(double(expandedMap), goalpx);
tElapsed = toc;
disp(['........ a.i: Transform computed in ', tElapsed])

disp('.... b: Computing Path')
% 1 at the end means that the dtransform was already computed (doesn't recompute)
tic
path = PathPlanning.findPath(double(dtransform), start, goalpx,1); 
tElapsed = toc;
disp(['........ b.i: Path Found in ', tElapsed])
%% Display Output
%%
disp('4. Creating Graphical Representation')


RGB = zeros(100,100,3);
RGB(:,:,1)   = occupancyGrid;
RGB(:,:,2)   = ones(100,100) - occupancyGrid;
pathDisp = 2*path/100;

figure
subplot(111)
expandedMap = flipud(expandedMap);
idisp (expandedMap,'xydata',{[0,2] [2,0]},'ynormal')
hold on
plot(pathDisp(:,1),pathDisp(:,2))
hold off

figure
subplot(211)
idisp (dtransform,'xydata',{[0,2] [2,0]},'ynormal')
hold off

figure
idisp (RGB,'xydata',{[0,2] [2,0]},'ynormal')
hold on
plot(pathDisp(:,1),pathDisp(:,2))
hold off
%% Pure Pursuit


d = 0.01;
dt = 0.25;

figure
rob = robotSim([x, y, theta],dt);
first = 1;
tic
for point = pathDisp.'    
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