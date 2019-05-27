function [d,dtheta] = encoderToPose(dTicks, q)
    ticksPerRev = 320;

    % Get distance travelled
    d = 2 * pi * 0.0325 * dTicks / ticksPerRev;
    
    % Calculate changes in values
    dtheta = (d(2) - d(1)) / 0.16;
    d = mean(d);
%     dx = dc * cos(q(3));
%     dy = dc * sin(q(3));
    
%     dq = [dx; dy; dtheta];

end