function vel = control(q, point)
    % Inputs:
    % q is the initial configuration vector (x, y, theta) in units of metres and radians
    % point is the vector (x, y) specifying the goal point of the robot
    Kh = 6; % Heading Gain
    Kv = 0.5; % Velocity Gain
    
    
    v = Kv * sqrt((point(1) - q(1))^2 + (point(2) - q(2))^2); % Velocity Controller
    
    
    desiredTheta = atan2((point(2) - q(2)),(point(1)-q(1))); % Calculating the angle of the goal
    
    omega = Kh * (mod((desiredTheta - q(3))+pi, 2*pi) - pi); 
    
    vel = vw2wheels([v,omega]);
    
end
