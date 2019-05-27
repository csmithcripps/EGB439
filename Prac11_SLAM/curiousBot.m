function [posx,posy] = curiousBot(i,mu,idMap,pb,S,R,Q)
    if (length(idMap)<i)
        
        pb.stop()
        pb.resetEncoder()
        prevEncoder = 0;
        newBeacon = false;
        while (~newBeacon)
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
                    disp("Update")
                    disp(idList(j,1))
                    [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
                    idNumber = find(idMap == idList(j,1));
                    [mu,S] = update_step(idNumber,[r;b],mu,S,Q);
                else
                    disp("New Landmark")
                    disp(idList(j,1))
                    [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
                    [mu,S] = initLandmarks([r;b],Q,mu,S);
                    idMap = [idMap; idList(j,1)];
                    newBeacon = true;
                end
            end

            %% Control Robot
            % Drive toward goal 
            vel = [10,-10];
            pb.setVelocity(vel)   

            plotSLAM(mu,S)
        end
    else
        bix = mu(3+ 2*(i-1) +1);
        biy = mu(3+ 2*(i-1) +2);
        
        r = sqrt((bix - mu(1))^2 + (biy - mu(2))^2);
        b = atan2(biy - mu(2), bix - mu(1));
    end
    posR = r-0.1;
    posx = posR*sin(b);
    posy = posR*cos(b);

end