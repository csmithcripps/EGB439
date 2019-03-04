% pb = PiBot('172.19.232.105');


x1 = 1;
y1 = 1;
theta = -45;
l = 0.2;
w = 0.1;

R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
triangle = [0 0; -l +w; -l -w]



% xm = x1-l* cos(theta);
% ym = y1-l* sin(theta);
% 
% x2 = xm - (w/2)* cos(theta);
% y2 = ym + (w/2)* sin(theta);
% 
% x3 = xm + (w/2)* cos(theta);
% y3 = ym - (w/2)* sin(theta);

PoseTriangle = triangle*R + [x1,y1; x1,y1; x1,y1]


figure
hold on;
% line([x1,x2], [y1, y2], 'Color', 'r');
% line([x2,x3], [y2, y3], 'Color', 'r');
% line([x3,x1], [y3, y1], 'Color', 'r');

fill(PoseTriangle(:,1),PoseTriangle(:,2),'r');
axis([0,2,0,2]);