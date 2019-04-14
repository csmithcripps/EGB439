function qnew = qupdate(q, vel, dt)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and
    % radians
    % vel is the velocity vector (vleft, vright) each in the range -100 to
    % +100
    % dt is the length of the integration timestep in units of seconds
    % Return:
    % qnew is the new configuration vector vector (x, y, theta) in units of
    % metres and radians at the end of the time interval.
    
    %Finding q*
    qdot_current = qdot(q, vel);
    
    % Finding qnew
    qnew = dt*qdot_current + q;
    
    %Ensuring theta doesn't exceed pi or -pi
    theta = qnew(3);
    while theta < pi
        theta = theta + 2*pi;
    end
    
    while theta > pi
        theta = theta - 2*pi; 
    end
    qnew(3) = theta;
    
end