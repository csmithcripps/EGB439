function myFunction(q0, sensorFunction)
    % q0 is a 3x1 vector giving the initial configuration of the robot in units of metres and degrees
    % sensorFunction is a function that accepts the robot configuration and 
    % returns a vector containing the left and right sensor values
    % eg. sensors = sensorFunction( q )
    % where sensors is a 2x1 vector containing the left and right sensor readings
    
    % Setup initial values
    q = q0;
    points = [q];
    sensors = sensorFunction(q);
    speeds = [0;0];
    % Run simulation of motion until within range
    while sensors(1) <= 0.99 && sensors(2) <= 0.99
        %Get sensor Values
        sensors = sensorFunction(q);
        
        %Slow down as you approach the goal
        speeds = 1-flipud(sensors);
        
        %Update position and store
        q = qupdate(q, speeds);
        points = [points q];

    end
        
    plot(points(1,:),points(2,:))
        
        
    
    
    
    % sensors = sensorFunction(q)
    % compute wheel speeds
    % q = qupdate(q, speeds)
    
    % plot()
end
    