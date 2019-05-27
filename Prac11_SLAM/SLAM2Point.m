function [mu, S, idMap] = SLAM2Point(mu,S,idMap,goal,pb,R,Q)
    r = sqrt((goal(1) - mu(1))^2 + (goal(2) - mu(2))^2);
    pb.stop()
    pb.resetEncoder()
    prevEncoder = 0;
    while(r>0.1)
        %% Read Inputs
        encoder = pb.getEncoder;
        cam = pb.getImage();
        %% Predict  
        % Update Encoder    
        dTicks = encoder - prevEncoder;
        prevEncoder = encoder;
        dq = encoderToPose(dTicks, mu(1:3));
        [mu,S] = predict_step(mu,S,dq,R);

        %% Update
        idList = idBeacon(cam);
        for j = 1:size(idList,1)
            if sum(idMap == idList(j))
                [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
                idNumber = find(idMap == idList(j,1));
                [mu,S] = update_step(idNumber,[r;b],mu,S,Q);
            else
                [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
                [mu,S] = initLandmarks([r;b],Q,mu,S);
                idMap = [idMap; idList(j,1)];
            end
        end

        %% Control Robot
        % Drive toward goal
        vel = control.driveToPoint(mu(1:3),goal(1:2),0.1,0.08);  % compute the wheel speeds given the current configuration
        pb.setVelocity(vel)   

        plotSLAM(mu,S)
        r = sqrt((goal(1) - mu(1))^2 + (goal(2) - mu(2))^2);
    end
end