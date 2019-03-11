x = 2.1;
y = 1.3;
theta = 40;
% Write the configuration of the vehicle (1x3 vector) in units of metres and degrees.  
Q = [x y theta]

% Write a 3x3 homogeneous transformation matrix that describes the pose of this robot 
% with respect to the world coordinate frame.
TR = [cosd(theta), -sind(theta), x;...
      sind(theta),  cosd(theta), y;...
      0,            0,           1]

% Write a 3x3 homogeneous transformation matrix that describes the pose of the sensor 
% with respect to the world coordinate frame.
thetaS = theta - 5;

TS = TR* [cosd(-5), -sind(-5),  0.4;...
          sind(-5),  cosd(-5), -0.15;...
          0,            0,      1]
% Write the position of the navigation target relative to the sensor in polar 
% coordinate form as a 2x1 vector  (in units of metres and degrees, respectively)
g = TS^(-1)*[3.1;3.2;1]
r = sqrt(g(1)^2 + g(2)^2);
gtheta = atan2d(g(2),g(1));

PP = [r; ...
      gtheta]