function vel = control(q, point)
    % Inputs:
    % q is the initial configuration vector (x, y, theta) in units of metres and radians
    % point is the vector (x, y) specifying the point point of the robot
    
    % Constants
    heading_constant = 0.43;
    velocity_constant = 0.15;
    speed_constant = 0.03;
    d = 0.01;
    dt = 0.2;

    % Setup e value
    global e_int;
    if isempty(e_int)
       e_int = 0;
    end

    % Get values
    point_x = point(1);
    point_y = point(2);
    x = q(1);
    y = q(2);
    theta = q(3);
    
    % Calculate angle difference
    theta_goal = atan2(point_y-y, point_x-x);
    h = theta_goal - theta;
    while (h >  pi); h = h-2*pi; end
    while (h < -pi); h = h+2*pi; end
    
    % Calculate turning speed
    w = heading_constant * h;
    w = min(w, 1);
    w = max(w, -1);
    
    % Calculate forward speed
    e = sqrt((point_x-x)^2 + (point_y-y)^2) - d;
    e_int = e_int + e * dt;
    v = velocity_constant * e + speed_constant * e_int;
    v = min(v, 0.5);
    
    % Calculate wheel speeds
    vw = [v w];
    vel = vw2wheels(vw);
    disp(vel);
    
end






