function [mu, S, idMap] = SLAM2Point(mu,S,idMap,goal,pb,R,Q)
    r = sqrt((goal(1) - mu(1))^2 + (goal(2) - mu(2))^2);
    pb.stop()
    pb.resetEncoder()
    prevEncoder = 0;
    while(r>0.15)
        %% Read Inputs
        encoder = pb.getEncoder;
        cam = pb.getImage();
        %% Predict  
        % Update Encoder    
        dTicks = encoder - prevEncoder;
        prevEncoder = encoder;
        [d,dth] = encoderToPose(dTicks, mu(1:3));
        [mu,S] = predict_step(mu,S,d,dth,R);

        %% Update
        idList = idBeacon(cam);
        for j = 1:size(idList,1)
            if sum(idMap == idList(j))
                [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
                idNumber = find(idMap == idList(j,1));
                [mu,S] = update_step(idNumber,[r;b],mu,S,Q);
            else
                disp("New Landmark")
                disp(idList(j,1))
                [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
                [mu,S] = initLandmarks([r;b],Q,mu,S);
                idMap = [idMap; idList(j,1)]
%                 pb.setLEDArray(
            end
        end

        %% Control Robot
        % Drive toward goal
        vel = control.driveToPoint(mu(1:3),goal(1:2),0.095,0.05);  % compute the wheel speeds given the current configuration
        pb.setVelocity(vel)   

        plotSLAM(mu,S,idMap)
        hold on
        plot(goal(1),goal(2),'kp')
        drawnow
        
        r = sqrt((goal(1) - mu(1))^2 + (goal(2) - mu(2))^2);
    end
    pb.stop()
    disp("SLAMMMED")
end