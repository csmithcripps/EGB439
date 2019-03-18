function vel = driveToPoint(q,point)
%DRIVETOPOINT 
%   q     = [x,y,theta] current pose
%   point = [x,y] goal point
%   vel   = [vleft,vright] (values from -100 to 100)

    Kh = 0.005;
    Kv = 0.005;
    
    
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

