%% Prac 5

addpath('~/Documents/EGB439/Robot_Functions')

pb = PiBot('172.19.232.178','172.19.232.12',6);


img = pb.getLocalizerImage();

imgThreshhold = img > 20;
imgThreshhold(:,1:20) = 0;
imgThreshhold(:,end-20:end) = 0;
imgThreshhold(1:20,:) = 0;
imgThreshhold(end-20:end,:) = 0;

occupancyGrid = imresize(imgThreshhold, 1/5);

RGB = zeros(100,100,3);
RGB(:,:,1)   = occupancyGrid;
RGB(:,:,2)   = ones(100,100) - occupancyGrid;

idisp(RGB, 'xydata',{[0,2], [0,2]}) ;
