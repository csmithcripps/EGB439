function wheelVel = vw2wheels(vw)
    % Inputs:
    % vw is the velocity vector (v, omega) in units of metres/s and radians/s
    % Return:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100 to achieve
    % this velocity   
    
    W = 0.16; % Lateral Wheel Spacing
    d = 0.065; % Wheel Diameter
    
    vdelta = vw(2) * W;
    vlinear = [-1,1;1,1] * [vdelta ; 2 * vw(1)]; %[vl;vr]
    
    vleft = vlinear(1) * (2/(pi * d));
    vright = vlinear(2) * (2/(pi * d));
    
    wheelVel = [vleft,vright];
end

function vw = wheels2vw(wheelVel)
    % Inputs:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +cw100
    % Return:
    % vw is the resulting velocity vector (v, omega) in units of metres/s and radians/s
    
    W = 0.16; % Lateral Wheel Spacing
    vlinear = (wheelVel ./2) .* pi*(0.065); % Convert from wheelVel to linear Velocities.
    
    vdelta = vlinear(2) - vlinear(1);
    v = 0.5 * (vlinear(1) + vlinear(2));
    omega = vdelta/W;
    
    vw = [v , omega];
end

function qd = qdot(q, vel)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (vleft, vright) each in the range -100 to +100
    % Return:
    % qd is the vector (xdot, ydot, thetadot) in units of metres/s and radians/s
    
    vw = wheels2vw(vel);
    xdot = vw(1)*cos(q(3));
    ydot = vw(1)*sin(q(3));
    
    qd = [xdot,ydot,vw(2)];
    
end

function qnew = qupdate(q, vel, dt)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (vleft, vright) each in the range -100 to +100
    % dt is the length of the integration timestep in units of seconds
    % Return:
    % qnew is the new configuration vector vector (x, y, theta) in units of metres and radians at the
    % end of the time interval.
    qd = qdot(q,vel);
    qnew = q + qd * dt;
end

function vel = control(q, point)
    % Inputs:
    % q is the initial configuration vector (x, y, theta) in units of metres and radians
    % point is the vector (x, y) specifying the goal point of the robot
    Kh = 0.5;
    Kv = 0.5;
    
    
    v = Kv * sqrt((point(1) - q(1))^2 + (point(2) - q(2))^2);
    desiredTheta = atan2((point(2) - q(2)),(point(1)-q(1)));
    deltatheta = desiredTheta - q(3);
    while deltatheta > 180
        deltatheta = deltatheta - 180;
    end
    while deltatheta < -180
        deltatheta = deltatheta + 180;
    end
    
    omega = Kh * deltatheta;
    
    vel = vw2wheels([v,omega]);
    
end
