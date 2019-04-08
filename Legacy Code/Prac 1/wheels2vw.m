function vw = wheels2vw(wheelVel)
    % Inputs:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100
    % Return:
    % vw is the resulting velocity vector (v, omega) in units of metres/s and radians/s
    
    W = 0.16; % Lateral Wheel Spacing
    vlinear = (wheelVel ./2) .* pi*(0.065); % Convert from wheelVel to linear Velocities.
    
    vdelta = vlinear(2) - vlinear(1);
    
    v = 0.5 * (vlinear(1) + vlinear(2));
    omega = vdelta/W;
    
    vw = [v , omega];
end