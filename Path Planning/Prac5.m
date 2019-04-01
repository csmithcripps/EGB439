%% Prac 5

addpath('~/Documents/EGB439/Robot_Functions')

pb = PiBot('172.19.232.178','172.19.232.12',6);


pose = pb.getLocalizerPose.pose;
x = pose.x;
y = 2-pose.y;
img = pb.getLocalizerImage();

imgThreshhold = img > 20;
imgThreshhold(:,1:20) = 0;
imgThreshhold(:,end-20:end) = 0;
imgThreshhold(1:20,:) = 0;
imgThreshhold(end-20:end,:) = 0;

occupancyGrid = imresize(imgThreshhold, 1/5);

%Remove Robot
if x ~= 0 && y ~= 0
    x1   = round(max(x*50 - 10,1));
    y1   = round(min(y*50 - 10,100));
    x2   = round(max(x*50 + 10,1));
    y2   = round(min(y*50 + 10,100));
    occupancyGrid(y1:y2,x1:x2) = 0;
else
    x = 90;
    y = 90;
end

RGB = zeros(100,100,3);
RGB(:,:,1)   = occupancyGrid;
RGB(:,:,2)   = ones(100,100) - occupancyGrid;

hold on
plot(x,y)
% imshow(RGB)
% idisp(RGB, 'xydata',{[0,2], [0,2]});
pause(0.2)

goal = ([0.3,0.3]/2) * 100;

dtransform = myFunction('dxform', double(occupancyGrid), goal);
figure
idisp(dtransform)

path = myFunction('path', double(occupancyGrid), [x,y], goal)



figure
idisp (RGB,'xydata',{[0,2] [2,0]},'ynormal')
