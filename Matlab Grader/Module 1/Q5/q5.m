clearvars
v = -0.7;
gamma = 15;
dt = 0.05;
q0 = [0,0,0];
t = 0:dt:5;

Hl = [ 1   0    -0.5;
       0   1    0.85;
       0   0       1];
   
Hr = [ 1   0    -0.5;
       0   1   -0.85;
       0   0       1];

q = zeros(length(t),3);
ql = zeros(length(t),3);
qr = zeros(length(t),3);

ql(1,:) = [-0.5,0.85,0];
qr(1,:) = [-0.5,-0.85,0];

carBox = [ 3.5  0.85 0;
        3.5 -0.85 0;
       -0.5 -0.85 0;
       -0.5  0.85 0;
        3.5  0.85 0];



for i = 2:length(t)
    q(i,:) = qupdate(q(i-1,:),v,gamma,dt);
    ql(i,:) = Hl* [q(i,1:2) 1]';
    qr(i,:) = Hr* [q(i,1:2) 1]';
    
    
%     hold on
%     plot( q(1:i,1), q(1:i,2), 'r')
%     plot(ql(1:i,1),ql(1:i,2), 'b', 'Tag', 'Left')
%     plot(qr(1:i,1),qr(1:i,2), 'y', 'Tag', 'Right')
%     axis 'square'
    axis([-4 0 -1 3])
%     pause(dt)
end

ql1 = ql(20,:);
qr1 = qr(20,:);


hold on
plot(  carBox(:,1),carBox(:,2),'kp-', 'Tag', 'Car')
plot(  ql(1:i,1),ql(1:i,2), 'b-.', 'Tag', 'Left')
plot(  qr(1:i,1),qr(1:i,2), 'r-.', 'Tag', 'Right')
plot(  [ql1(1) qr1(1)], [ql1(2) qr1(2)], 'c', 'Tag', '1Second')
axis 'square'


function qd = qdot(q, v, gamma)
    L = 3;

    xdot = v*cos(q(3));
    ydot = v*sin(q(3));
    thetadot = (v/L) * tand(gamma);


    qd = [xdot,ydot,thetadot];

end

function qnew = qupdate(q, v, gamma, dt)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (vleft, vright) each in the range -100 to +100
    % dt is the length of the integration timestep in units of seconds
    % Return:
    % qnew is the new configuration vector vector (x, y, theta) in units of metres and radians at the
    % end of the time interval.
    qd = qdot(q,v,gamma);
    qnew = q + qd * dt;
end


