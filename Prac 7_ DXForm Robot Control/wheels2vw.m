function vw = wheels2vw(wheelVel)
    % Inputs:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the 
    % range -100 to +100
    % Return:
    % vw is the resulting velocity vector (v, omega) in units of metres/s
    % and radians/s
    
    %Wheel and Base constants
    wheel_radius = 0.032; %wheel radius in metres
    base_radius = 0.0725; %base radius in metres

    %Converting power to radians/second
    ang_velocity = (wheelVel*0.5) * (pi/30);
    
    %Finding wheel velocities
    wheel_velocities = wheel_radius * ang_velocity;
    right_velocity = wheel_velocities(2);
    left_velocity = wheel_velocities(1);
    
    %Calculating velocity and omega
    velocity = 0.5 * (right_velocity + left_velocity);
    omega = (right_velocity-left_velocity) / (2*base_radius);
    vw = [velocity, omega];

end