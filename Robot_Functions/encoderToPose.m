function dq = encoderToPose(dTicks, q)
    ticksPerRev = 320;

    % Get distance travelled
    d = 2 * pi * 0.0325 * dTicks / ticksPerRev;
    
    % Calculate changes in values
    dc = mean(d);
    dx = dc * cos(q(3));
    dy = dc * sin(q(3));
    dtheta = (d(2) - d(1)) / 0.16;
    
    dq = [dx; dy; dtheta];

end