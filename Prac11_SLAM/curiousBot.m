function [posx,posy] = curiousBot(i,mu,idMap,pb)
    if (length(idMap)<i)
        tic
        newBeacon = false;
        while (~newBeacon)
            %% Read Inputs
            cam = pb.getImage();

            %% Update
            idList = idBeacon(cam);
            for j = 1:size(idList,1)
                if ~sum(idMap == idlist(j))
                    [r,b] = getBeaconRangeBearing(idList(j,2),idList(j,3));
                    newBeacon = true;
                end
            end

            %% Control Robot
            vel = [10,-10];
            pb.setVelocity(vel)
        end
        pb.stop()
        t1 = toc
        tic
        t2 = 0
        while (t2<t1)    
            vel = [-10,10];
            pb.setVelocity(vel)
            t2 = toc
        end
    else
        bix = mu(3+ 2*(i-1) +1);
        biy = mu(3+ 2*(i-1) +2);
        
        r = sqrt((bix - mu(1))^2 + (biy - mu(2))^2);
        b = atan2(biy - mu(2), bix - mu(1));
    end
    posR = r-0.1;
    posx = posR*sin(beta);
    posy = posR*cos(beta);

end