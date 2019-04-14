function wheelVel = vw2wheels(vw)
    % Inputs:
    % vw is the velocity vector (v, omega) in units of metres/s and
    % radians/s
    % Return:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the
    % range -100 to +100 to achieve
    % this velocity
    
    % Wheel and Base constants
    wheel_radius = 0.032; %measured in metres
    base_radius  = 0.0725; %measured in metres
 
    % Find velocity & omega values
    velocity = vw(1);
    omega = vw(2);
    
    % Calculate velocities of each wheel by simultaneous equtions
    left_velocity = velocity - omega*base_radius;
    right_velocity = 2*velocity - left_velocity;
    wheel_velocities = [left_velocity, right_velocity];
    
    % Calculate angular velocity of the wheels
    ang_velocity = wheel_velocities/wheel_radius;
    
    % Convert radians/second to power
    wheelVel = ang_velocity / (pi/30) * 2;
    
    % Apply limits
    for i = 1:2
        if wheelVel(i) > 100
            wheelVel(i) = 100;

        elseif wheelVel(i) < 100
            wheelVel(i) = -100;
        end
    end
end