function q = updatePose(q, ticks)
    wheelBase = 0.16;
    distance = encoderToMetre(ticks);
    
    dc = mean(distance);
    dx = dc * cos(q(3));
    dy = dc * sin(q(3));
    dtheta = (distance(2) - distance(1))/wheelBase;
    
    q = q + [dx, dy, dtheta];
end

