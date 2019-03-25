function qd = qdot(q, vel)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (vleft, vright) each in the range -100 to +100
    % Return:
    % qd is the vector (xdot, ydot, thetadot) in units of metres/s and radians/s
    
    vw = wheels2vw(vel);
    xdot = vw(1)*cos(q(3));
    ydot = vw(1)*sin(q(3));
    thetadot = vw(2);
    
    
    qd = [xdot,ydot,thetadot];
    
end